//! The subset of KiCad's DRC rule condition language the week-1 table
//! evaluates.
//!
//! Grammar (precedence low to high):
//!
//! ```text
//! or      := and ( "||" and )*
//! and     := unary ( "&&" unary )*
//! unary   := "!" unary | primary
//! primary := "(" or ")" | call | operand ( "==" | "!=" ) operand
//! call    := ("A" | "B") "." ("intersectsCourtyard" | "enclosedByArea") "(" string ")"
//! operand := ("A" | "B") "." ("NetName" | "NetClass" | "Type" | "Reference") | string
//! ```
//!
//! Anything else — another property, function, operator or a numeric
//! comparison — is refused at compile time with a reason, and the rule that
//! carried it is reported inert rather than half-evaluated.
//!
//! String comparison follows KiCad's `LIBEVAL::VALUE::EqualTo`: a literal
//! containing `*` or `?` is a wildcard pattern, and comparison is
//! case-insensitive.

use crate::api::FootprintId;

/// Which side of a two-item rule a term refers to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Obj {
    A,
    B,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Prop {
    NetName,
    NetClass,
    Type,
    Reference,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum Operand {
    Prop(Obj, Prop),
    Lit(String),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum Expr {
    True,
    Or(Box<Expr>, Box<Expr>),
    And(Box<Expr>, Box<Expr>),
    Not(Box<Expr>),
    /// `lhs == rhs`, or `!=` when `negated`.
    Cmp {
        lhs: Operand,
        rhs: Operand,
        negated: bool,
    },
    IntersectsCourtyard(Obj, FootprintId),
    EnclosedByArea(Obj, String),
}

/// The property values of one rule subject, as condition evaluation sees
/// them. `None` stands for the missing `B` of a single-item constraint.
pub(crate) trait SubjectView {
    fn prop(&self, prop: Prop) -> &str;
    fn intersects_courtyard(&self, footprint: FootprintId) -> bool;
    fn enclosed_by_area(&self, area: &str) -> bool;
}

impl Expr {
    pub(crate) fn eval(&self, a: &dyn SubjectView, b: Option<&dyn SubjectView>) -> bool {
        let pick = |o: Obj| match o {
            Obj::A => Some(a),
            Obj::B => b,
        };
        match self {
            Expr::True => true,
            Expr::Or(l, r) => l.eval(a, b) || r.eval(a, b),
            Expr::And(l, r) => l.eval(a, b) && r.eval(a, b),
            Expr::Not(e) => !e.eval(a, b),
            Expr::Cmp { lhs, rhs, negated } => {
                let value = |op: &Operand| -> (String, bool) {
                    match op {
                        Operand::Lit(s) => (s.clone(), true),
                        Operand::Prop(o, p) => (
                            pick(*o).map(|s| s.prop(*p).to_string()).unwrap_or_default(),
                            false,
                        ),
                    }
                };
                let (l, l_lit) = value(lhs);
                let (r, r_lit) = value(rhs);
                let equal = if r_lit && is_wildcard(&r) {
                    wildcard_matches(&r, &l, false)
                } else if l_lit && is_wildcard(&l) {
                    wildcard_matches(&l, &r, false)
                } else {
                    l.to_lowercase() == r.to_lowercase()
                };
                equal != *negated
            }
            Expr::IntersectsCourtyard(o, fp) => {
                pick(*o).is_some_and(|s| s.intersects_courtyard(*fp))
            }
            Expr::EnclosedByArea(o, area) => pick(*o).is_some_and(|s| s.enclosed_by_area(area)),
        }
    }
}

fn is_wildcard(s: &str) -> bool {
    s.contains('*') || s.contains('?')
}

/// KiCad wildcard matching: `*` is any run including none, `?` exactly one
/// character. Iterative, so `*`-heavy patterns cannot exhaust the stack.
pub(crate) fn wildcard_matches(pattern: &str, name: &str, case_sensitive: bool) -> bool {
    let fold = |s: &str| -> Vec<char> {
        if case_sensitive {
            s.chars().collect()
        } else {
            s.chars().flat_map(char::to_lowercase).collect()
        }
    };
    wildcard_match_chars(&fold(pattern), &fold(name))
}

/// [`wildcard_matches`] on characters already collected (and folded), for
/// callers matching many names against many patterns.
pub(crate) fn wildcard_match_chars(pat: &[char], txt: &[char]) -> bool {
    let (mut p, mut t) = (0usize, 0usize);
    let (mut star, mut retry) = (None, 0usize);
    while t < txt.len() {
        if p < pat.len() && (pat[p] == '?' || pat[p] == txt[t]) {
            p += 1;
            t += 1;
        } else if p < pat.len() && pat[p] == '*' {
            star = Some(p);
            retry = t;
            p += 1;
        } else if let Some(s) = star {
            p = s + 1;
            retry += 1;
            t = retry;
        } else {
            return false;
        }
    }
    while p < pat.len() && pat[p] == '*' {
        p += 1;
    }
    p == pat.len()
}

// ─── Parsing ─────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq, Eq)]
enum Tok {
    Ident(String),
    Str(String),
    And,
    Or,
    Not,
    Eq,
    Ne,
    LParen,
    RParen,
}

fn tokenize(src: &str) -> Result<Vec<Tok>, String> {
    let chars: Vec<char> = src.chars().collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        let next = chars.get(i + 1).copied();
        match c {
            c if c.is_whitespace() => i += 1,
            '(' => {
                out.push(Tok::LParen);
                i += 1;
            }
            ')' => {
                out.push(Tok::RParen);
                i += 1;
            }
            '&' if next == Some('&') => {
                out.push(Tok::And);
                i += 2;
            }
            '|' if next == Some('|') => {
                out.push(Tok::Or);
                i += 2;
            }
            '=' if next == Some('=') => {
                out.push(Tok::Eq);
                i += 2;
            }
            '!' if next == Some('=') => {
                out.push(Tok::Ne);
                i += 2;
            }
            '!' => {
                out.push(Tok::Not);
                i += 1;
            }
            '\'' => {
                let start = i + 1;
                let end = (start..chars.len())
                    .find(|&j| chars[j] == '\'')
                    .ok_or_else(|| "unterminated string".to_string())?;
                out.push(Tok::Str(chars[start..end].iter().collect()));
                i = end + 1;
            }
            c if c.is_ascii_alphabetic() || c == '_' => {
                let start = i;
                while i < chars.len()
                    && (chars[i].is_ascii_alphanumeric() || chars[i] == '_' || chars[i] == '.')
                {
                    i += 1;
                }
                out.push(Tok::Ident(chars[start..i].iter().collect()));
            }
            other => return Err(format!("unsupported character '{other}'")),
        }
    }
    Ok(out)
}

/// What the parser needs from the board to resolve names at compile time.
pub(crate) trait NameResolver {
    /// The footprint with this reference, if the board has exactly one.
    fn footprint(&self, reference: &str) -> Option<FootprintId>;
    /// Whether a rule area (or named zone) with this name exists.
    fn has_area(&self, name: &str) -> bool;
}

/// Compile a condition string. `Err` carries the reason the rule is inert.
pub(crate) fn parse_condition(src: &str, names: &dyn NameResolver) -> Result<Expr, String> {
    let toks = tokenize(src)?;
    let mut p = Parser {
        toks,
        pos: 0,
        names,
    };
    let e = p.or()?;
    if p.pos != p.toks.len() {
        return Err(format!("unexpected trailing input at token {}", p.pos));
    }
    Ok(e)
}

struct Parser<'a> {
    toks: Vec<Tok>,
    pos: usize,
    names: &'a dyn NameResolver,
}

impl Parser<'_> {
    fn peek(&self) -> Option<&Tok> {
        self.toks.get(self.pos)
    }

    fn bump(&mut self) -> Option<Tok> {
        let t = self.toks.get(self.pos).cloned();
        self.pos += 1;
        t
    }

    fn expect(&mut self, want: &Tok) -> Result<(), String> {
        match self.bump() {
            Some(ref t) if t == want => Ok(()),
            other => Err(format!("expected {want:?}, found {other:?}")),
        }
    }

    fn or(&mut self) -> Result<Expr, String> {
        let mut e = self.and()?;
        while self.peek() == Some(&Tok::Or) {
            self.pos += 1;
            e = Expr::Or(Box::new(e), Box::new(self.and()?));
        }
        Ok(e)
    }

    fn and(&mut self) -> Result<Expr, String> {
        let mut e = self.unary()?;
        while self.peek() == Some(&Tok::And) {
            self.pos += 1;
            e = Expr::And(Box::new(e), Box::new(self.unary()?));
        }
        Ok(e)
    }

    fn unary(&mut self) -> Result<Expr, String> {
        if self.peek() == Some(&Tok::Not) {
            self.pos += 1;
            return Ok(Expr::Not(Box::new(self.unary()?)));
        }
        self.primary()
    }

    fn primary(&mut self) -> Result<Expr, String> {
        if self.peek() == Some(&Tok::LParen) {
            self.pos += 1;
            let e = self.or()?;
            self.expect(&Tok::RParen)?;
            return Ok(e);
        }
        // A call is an identifier immediately followed by "(".
        if let (Some(Tok::Ident(name)), Some(Tok::LParen)) =
            (self.toks.get(self.pos), self.toks.get(self.pos + 1))
        {
            let name = name.clone();
            self.pos += 2;
            let arg = match self.bump() {
                Some(Tok::Str(s)) => s,
                other => {
                    return Err(format!(
                        "{name}() expects one string argument, found {other:?}"
                    ))
                }
            };
            self.expect(&Tok::RParen)?;
            let (obj, func) = split_member(&name)?;
            return match func {
                "intersectsCourtyard" => match self.names.footprint(&arg) {
                    Some(fp) => Ok(Expr::IntersectsCourtyard(obj, fp)),
                    None => Err(format!(
                        "footprint '{arg}' is not on the board (or is not unique)"
                    )),
                },
                "enclosedByArea" => {
                    if self.names.has_area(&arg) {
                        Ok(Expr::EnclosedByArea(obj, arg))
                    } else {
                        Err(format!("area '{arg}' is not on the board"))
                    }
                }
                other => Err(format!("function '{other}()' is not supported")),
            };
        }
        let lhs = self.operand()?;
        let negated = match self.bump() {
            Some(Tok::Eq) => false,
            Some(Tok::Ne) => true,
            other => return Err(format!("expected == or !=, found {other:?}")),
        };
        let rhs = self.operand()?;
        Ok(Expr::Cmp { lhs, rhs, negated })
    }

    fn operand(&mut self) -> Result<Operand, String> {
        match self.bump() {
            Some(Tok::Str(s)) => Ok(Operand::Lit(s)),
            Some(Tok::Ident(name)) => {
                let (obj, prop) = split_member(&name)?;
                let prop = match prop {
                    "NetName" => Prop::NetName,
                    "NetClass" => Prop::NetClass,
                    "Type" => Prop::Type,
                    "Reference" => Prop::Reference,
                    other => return Err(format!("property '{other}' is not supported")),
                };
                Ok(Operand::Prop(obj, prop))
            }
            other => Err(format!("expected a property or string, found {other:?}")),
        }
    }
}

fn split_member(name: &str) -> Result<(Obj, &str), String> {
    match name.split_once('.') {
        Some(("A", member)) => Ok((Obj::A, member)),
        Some(("B", member)) => Ok((Obj::B, member)),
        _ => Err(format!("'{name}' is not an A./B. member")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Names;
    impl NameResolver for Names {
        fn footprint(&self, reference: &str) -> Option<FootprintId> {
            (reference == "IC7").then_some(FootprintId(7))
        }
        fn has_area(&self, name: &str) -> bool {
            name == "zone_a"
        }
    }

    struct Subj {
        net: &'static str,
        class: &'static str,
        kind: &'static str,
        reference: &'static str,
        in_ic7: bool,
    }
    impl SubjectView for Subj {
        fn prop(&self, prop: Prop) -> &str {
            match prop {
                Prop::NetName => self.net,
                Prop::NetClass => self.class,
                Prop::Type => self.kind,
                Prop::Reference => self.reference,
            }
        }
        fn intersects_courtyard(&self, fp: FootprintId) -> bool {
            self.in_ic7 && fp == FootprintId(7)
        }
        fn enclosed_by_area(&self, _area: &str) -> bool {
            false
        }
    }

    fn subj(net: &'static str, kind: &'static str, in_ic7: bool) -> Subj {
        Subj {
            net,
            class: "Default",
            kind,
            reference: "",
            in_ic7,
        }
    }

    #[test]
    fn wildcards_match_like_kicad() {
        assert!(wildcard_matches("BUS_D?", "BUS_D3", true));
        assert!(!wildcard_matches("BUS_D?", "BUS_D10", true));
        assert!(wildcard_matches("*_P", "USB_D_P", true));
        assert!(wildcard_matches("usb_*", "USB_D_P", false));
        assert!(!wildcard_matches("usb_*", "USB_D_P", true));
        assert!(wildcard_matches("**a**", "xxaxx", true));
    }

    #[test]
    fn courtyard_pair_condition_evaluates_both_sides() {
        let e = parse_condition(
            "A.intersectsCourtyard('IC7') && B.intersectsCourtyard('IC7')",
            &Names,
        )
        .unwrap();
        let inside = subj("X", "Track", true);
        let outside = subj("Y", "Pad", false);
        assert!(e.eval(&inside, Some(&inside)));
        assert!(!e.eval(&inside, Some(&outside)));
        assert!(!e.eval(&inside, None));
    }

    #[test]
    fn negation_precedence_and_comparisons() {
        let e =
            parse_condition("A.Type == 'Via' && !A.intersectsCourtyard('IC7')", &Names).unwrap();
        assert!(e.eval(&subj("N", "Via", false), None));
        assert!(!e.eval(&subj("N", "Via", true), None));
        assert!(!e.eval(&subj("N", "Track", false), None));
        let e = parse_condition(
            "(A.NetName == 'bus_d?' || A.NetName == 'CLK') && A.Type != 'Pad'",
            &Names,
        )
        .unwrap();
        assert!(e.eval(&subj("BUS_D1", "Track", false), None));
        assert!(!e.eval(&subj("BUS_D1", "Pad", false), None));
    }

    #[test]
    fn unknown_names_and_constructs_are_refused_with_reasons() {
        let err = parse_condition("A.intersectsCourtyard('U9')", &Names).unwrap_err();
        assert!(err.contains("U9"), "{err}");
        let err = parse_condition("A.enclosedByArea('gone')", &Names).unwrap_err();
        assert!(err.contains("gone"), "{err}");
        assert!(parse_condition("A.enclosedByArea('zone_a')", &Names).is_ok());
        assert!(parse_condition("A.Width > 0.1", &Names).is_err());
        assert!(parse_condition("A.isPlated()", &Names).is_err());
        assert!(parse_condition("A.NetName == 'X' &&", &Names).is_err());
    }
}
