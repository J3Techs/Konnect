//! Net class resolution from a project's `net_settings`, as KiCad 10 does
//! it (`NET_SETTINGS::GetEffectiveNetClass`), described from KiCad's
//! behaviour; no KiCad code is reused.
//!
//! For a net name:
//!
//! 1. explicit `netclass_assignments` entries (a class name or a list);
//! 2. otherwise every `netclass_patterns` entry that matches. A pattern
//!    matches when it matches the **whole** name either as a regular
//!    expression (KiCad anchors it with `^…$`) or as a wildcard pattern
//!    (`*` any run, `?` one character, everything else literal). Patterns
//!    written in bus notation (`D[0..7]`, `GRP{A B}`) are expanded to their
//!    members first. KiCad keeps every match, not just the first;
//! 3. otherwise, when the name itself parses as a bus (a vector such as
//!    `D[0..3]` or a group such as `PREFIX{A B}`), its members are resolved
//!    in turn, and when all of them resolve to one class other than
//!    `Default`, the net takes it. A group's prefix and anything after its
//!    closing brace are dropped. This is how a net named
//!    `Net-(U9-A{slash}B)` (KiCad escapes `/` in a pin or label name as
//!    `{slash}`) ends up in a class whose pattern is `slash`;
//! 4. otherwise `Default`.
//!
//! When several classes match, KiCad builds a composite whose fields come
//! from the highest-precedence class that sets them (lowest `priority`,
//! then name). The rule table has one class per net, so it takes that
//! highest-precedence class and reports the net as ambiguous.

use super::NetClassRules;
use crate::api::ClassId;
use std::collections::BTreeMap;

/// One expanded pattern member: its anchored regex (when the text compiles
/// as one and is not plain literal text) and its wildcard characters.
struct Matcher {
    regex: Option<regex::Regex>,
    wildcard: Vec<char>,
}

impl Matcher {
    fn new(pattern: &str) -> Self {
        Matcher {
            regex: anchored_regex(pattern),
            wildcard: pattern.chars().collect(),
        }
    }

    fn matches(&self, name: &str, name_chars: &[char]) -> bool {
        super::wildcard_match_chars(&self.wildcard, name_chars)
            || self.regex.as_ref().is_some_and(|r| r.is_match(name))
    }
}

/// `^pattern$` as a regular expression, when the pattern holds regex
/// syntax (plain text anchored matches exactly what the wildcard matcher
/// does already) and compiles. KiCad drops a regex that does not compile.
fn anchored_regex(pattern: &str) -> Option<regex::Regex> {
    const META: &[char] = &[
        '.', '^', '$', '*', '+', '?', '(', ')', '[', ']', '{', '}', '|', '\\',
    ];
    if !pattern.contains(META) {
        return None;
    }
    let mut p = String::with_capacity(pattern.len() + 2);
    if !pattern.starts_with('^') {
        p.push('^');
    }
    p.push_str(pattern);
    if !pattern.ends_with('$') || pattern.ends_with("\\$") {
        p.push('$');
    }
    regex::Regex::new(&p).ok()
}

/// Resolves net names to classes from a project's `net_settings`.
pub(crate) struct NetclassResolver {
    patterns: Vec<(Matcher, ClassId)>,
    explicit: BTreeMap<String, Vec<ClassId>>,
    /// Precedence order: lower sorts first and wins.
    rank: Vec<(i64, String)>,
}

/// One net's resolution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Resolved {
    pub class: ClassId,
    /// More than one class matched (KiCad composites them).
    pub ambiguous: bool,
}

impl NetclassResolver {
    /// `patterns` and `explicit` name classes by id; `priorities` holds each
    /// class's `priority` from the project (`None` when absent), in class id
    /// order.
    pub(crate) fn new(
        classes: &[NetClassRules],
        priorities: &[Option<i64>],
        patterns: &[(String, ClassId)],
        explicit: BTreeMap<String, Vec<ClassId>>,
    ) -> Self {
        let mut expanded = Vec::new();
        for (p, id) in patterns {
            for member in bus_members(p).unwrap_or_else(|| vec![p.clone()]) {
                expanded.push((Matcher::new(&member), *id));
            }
        }
        let rank = classes
            .iter()
            .enumerate()
            .map(|(i, c)| {
                // Default (always id 0) has the lowest precedence.
                let p = match priorities.get(i).copied().flatten() {
                    _ if i == 0 => i64::MAX,
                    Some(p) => p,
                    None => i as i64,
                };
                (p, c.name.clone())
            })
            .collect();
        NetclassResolver {
            patterns: expanded,
            explicit,
            rank,
        }
    }

    /// The class of `name`.
    pub(crate) fn resolve(&self, name: &str) -> Resolved {
        self.resolve_depth(name, 0)
    }

    fn pick(&self, mut found: Vec<ClassId>) -> Option<Resolved> {
        found.sort_by(|a, b| self.rank[a.index()].cmp(&self.rank[b.index()]));
        found.dedup();
        let class = *found.first()?;
        Some(Resolved {
            class,
            ambiguous: found.len() > 1,
        })
    }

    fn resolve_depth(&self, name: &str, depth: u32) -> Resolved {
        const DEFAULT: Resolved = Resolved {
            class: ClassId(0),
            ambiguous: false,
        };
        if name.is_empty() {
            return DEFAULT;
        }
        if let Some(ids) = self.explicit.get(name) {
            if let Some(r) = self.pick(ids.clone()) {
                return r;
            }
        }
        let chars: Vec<char> = name.chars().collect();
        let found: Vec<ClassId> = self
            .patterns
            .iter()
            .filter(|(m, _)| m.matches(name, &chars))
            .map(|(_, id)| *id)
            .collect();
        if let Some(r) = self.pick(found) {
            return r;
        }
        // A bus: every member in one class other than Default.
        if depth < 4 {
            if let Some(members) = bus_members(name) {
                let mut class = None;
                for m in &members {
                    let r = self.resolve_depth(m, depth + 1);
                    if r.class == ClassId(0) || class.is_some_and(|c| c != r.class) {
                        return DEFAULT;
                    }
                    class = Some(r.class);
                }
                if let Some(class) = class {
                    return Resolved {
                        class,
                        ambiguous: false,
                    };
                }
            }
        }
        DEFAULT
    }
}

/// Every class a net belongs to, by name, under the same rules as
/// [`NetclassResolver`] but without choosing one: what a report of netclass
/// membership needs (`get_netclasses`), where the router needs one class.
///
/// It follows KiCad more closely in two places the rule table does not need:
///
/// - explicit `netclass_assignments` and matching patterns are combined,
///   not one in place of the other;
/// - a bus takes its members' class only when every member resolves to the
///   same class *object*. A member that matches one class setting every
///   field the Default would otherwise fill resolves to that class; a member
///   that matches several, or one that leaves a field to the Default, gets a
///   composite of its own, which no other member shares. So a bus of two or
///   more such members falls back to `Default`, and a one-member group (a
///   `{slash}` name) takes the member's composite.
pub struct NetclassMembership {
    /// Expanded pattern members, each with the index of its entry in the
    /// project's `netclass_patterns` and the class it names.
    patterns: Vec<(Matcher, usize, String)>,
    explicit: BTreeMap<String, Vec<String>>,
    /// Classes that set every field the Default would otherwise fill.
    complete: std::collections::BTreeSet<String>,
}

/// One net's membership.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Membership {
    /// Every class the net belongs to, sorted, without the Default KiCad adds
    /// to fill fields (an explicit or pattern match to `Default` is listed).
    /// Empty: the net is in `Default` alone.
    pub classes: Vec<String>,
    /// The bus members the classes came through, when the net matched
    /// nothing itself and took its members' class; empty otherwise.
    pub via_bus_members: Vec<String>,
    /// Indices of the `netclass_patterns` entries that decided it.
    pub entries: Vec<usize>,
}

impl NetclassMembership {
    /// `patterns` are the project's `netclass_patterns` entries in file
    /// order as (pattern, class name); `explicit` its `netclass_assignments`;
    /// `complete` the classes that set every field the Default would fill.
    pub fn new(
        patterns: &[(String, String)],
        explicit: BTreeMap<String, Vec<String>>,
        complete: std::collections::BTreeSet<String>,
    ) -> Self {
        let mut expanded = Vec::new();
        // An empty pattern fits only the empty name, which is never resolved.
        for (index, (p, class)) in patterns
            .iter()
            .enumerate()
            .filter(|(_, (p, _))| !p.is_empty())
        {
            for member in bus_members(p).unwrap_or_else(|| vec![p.clone()]) {
                expanded.push((Matcher::new(&member), index, class.clone()));
            }
        }
        NetclassMembership {
            patterns: expanded,
            explicit,
            complete,
        }
    }

    /// The classes of `name`.
    pub fn of(&self, name: &str) -> Membership {
        self.of_depth(name, 0)
    }

    /// What a member resolves to, as KiCad compares it: the class's name
    /// when it alone decides every field, otherwise a composite owned by
    /// this member alone.
    fn identity(&self, name: &str, m: &Membership) -> String {
        match m.classes.as_slice() {
            [] => "Default".to_string(),
            [one] if one == "Default" || self.complete.contains(one) => one.clone(),
            _ => format!("composite for {name}"),
        }
    }

    fn of_depth(&self, name: &str, depth: u32) -> Membership {
        if name.is_empty() {
            return Membership::default();
        }
        let mut classes: Vec<String> = self.explicit.get(name).cloned().unwrap_or_default();
        let chars: Vec<char> = name.chars().collect();
        let mut entries = Vec::new();
        for (m, index, class) in &self.patterns {
            if m.matches(name, &chars) {
                classes.push(class.clone());
                entries.push(*index);
            }
        }
        classes.sort();
        classes.dedup();
        entries.sort_unstable();
        entries.dedup();
        if !classes.is_empty() {
            return Membership {
                classes,
                via_bus_members: Vec::new(),
                entries,
            };
        }
        if depth >= 4 {
            return Membership::default();
        }
        let Some(members) = bus_members(name) else {
            return Membership::default();
        };
        let mut shared: Option<(String, Membership)> = None;
        let mut used = Vec::new();
        for member in members.iter().filter(|m| m.as_str() != name) {
            let got = self.of_depth(member, depth + 1);
            let id = self.identity(member, &got);
            match &shared {
                Some((first, _)) if *first != id => return Membership::default(),
                Some(_) => {}
                None => shared = Some((id, got.clone())),
            }
            used.extend(got.entries);
        }
        match shared {
            Some((id, got)) if id != "Default" => {
                used.sort_unstable();
                used.dedup();
                let mut via: Vec<String> = members.into_iter().filter(|m| m != name).collect();
                via.dedup();
                Membership {
                    classes: got.classes,
                    via_bus_members: via,
                    entries: used,
                }
            }
            _ => Membership::default(),
        }
    }
}

/// A character that makes the brace after it formatting (subscript,
/// superscript, overbar), not a bus group.
fn formatting(c: Option<char>) -> bool {
    matches!(c, Some('_' | '^' | '~'))
}

/// The members of `s` when it is a bus in KiCad's notation, recursively
/// expanded; `None` when it is not a bus.
pub(crate) fn bus_members(s: &str) -> Option<Vec<String>> {
    if let Some(v) = bus_vector(s) {
        return Some(v);
    }
    let group = bus_group(s)?;
    let mut out = Vec::new();
    for m in group {
        match bus_members(&m) {
            Some(inner) => out.extend(inner),
            None => out.push(m),
        }
    }
    Some(out)
}

/// `PREFIX[a..b]SUFFIX`: members `PREFIX` + each number + `SUFFIX`.
fn bus_vector(s: &str) -> Option<Vec<String>> {
    let open = s.find('[')?;
    let raw_prefix = &s[..open];
    let mut prev = None;
    for c in raw_prefix.chars() {
        if c == ' ' || c == ']' || (c == '{' && !formatting(prev)) {
            return None;
        }
        prev = Some(c);
    }
    // A formatting wrapper around the range (`D_{[1..2]}`) is dropped.
    let prefix = ["_{", "^{", "~{"]
        .iter()
        .find_map(|w| raw_prefix.strip_suffix(w))
        .unwrap_or(raw_prefix);
    let rest = &s[open + 1..];
    let close = rest.find(']')?;
    let (a, b) = rest[..close].split_once("..")?;
    if a.is_empty() || b.is_empty() || !a.bytes().all(|c| c.is_ascii_digit()) {
        return None;
    }
    if !b.bytes().all(|c| c.is_ascii_digit()) {
        return None;
    }
    let (begin, end): (u64, u64) = (a.parse().ok()?, b.parse().ok()?);
    if begin == end {
        return None;
    }
    let tail = &rest[close + 1..];
    if !tail
        .chars()
        .all(|c| matches!(c, '}' | '+' | '-' | 'P' | 'N'))
    {
        return None;
    }
    let suffix: String = tail.chars().filter(|&c| c != '}').collect();
    let width = if a.len() > 1 && a.starts_with('0') {
        a.len()
    } else if b.len() > 1 && b.starts_with('0') {
        b.len()
    } else {
        0
    };
    let (lo, hi) = (begin.min(end), begin.max(end));
    if hi - lo > 4096 {
        return None;
    }
    Some(
        (lo..=hi)
            .map(|n| format!("{prefix}{n:0width$}{suffix}"))
            .collect(),
    )
}

/// `PREFIX{A B,C}…`: members `A`, `B`, `C` (the prefix and anything after
/// the closing brace are dropped).
fn bus_group(s: &str) -> Option<Vec<String>> {
    let chars: Vec<char> = s.chars().collect();
    let mut i = 0;
    let mut prev = None;
    let mut escaped = false;
    // The prefix, up to a brace that is not formatting.
    loop {
        let c = *chars.get(i)?;
        if escaped {
            escaped = false;
        } else if c == '\\' {
            escaped = true;
        } else if c == '{' && !formatting(prev) {
            break;
        } else if c == ' ' || c == '[' || c == ']' {
            return None;
        }
        prev = Some(c);
        i += 1;
    }
    i += 1;
    let mut members = Vec::new();
    let mut cur = String::new();
    let mut depth = 0u32;
    let mut prev = None;
    while let Some(&c) = chars.get(i) {
        match c {
            '{' if formatting(prev) => {
                depth += 1;
                cur.push(c);
            }
            '{' => return None,
            '}' if depth > 0 => {
                depth -= 1;
                cur.push(c);
            }
            '}' => {
                if !cur.is_empty() {
                    members.push(cur);
                }
                return Some(members);
            }
            ' ' | ',' if depth == 0 => {
                if !cur.is_empty() {
                    members.push(std::mem::take(&mut cur));
                }
            }
            _ => cur.push(c),
        }
        prev = Some(c);
        i += 1;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn class(name: &str) -> NetClassRules {
        NetClassRules {
            name: name.to_string(),
            clearance: 1,
            track_width: 1,
            via_diameter: 2,
            via_drill: 1,
            diff_pair_width: None,
            diff_pair_gap: None,
        }
    }

    fn resolver(patterns: &[(&str, u16)]) -> NetclassResolver {
        let classes = [class("Default"), class("A"), class("B")];
        let patterns: Vec<(String, ClassId)> = patterns
            .iter()
            .map(|(p, c)| (p.to_string(), ClassId(*c)))
            .collect();
        NetclassResolver::new(&classes, &[None, None, None], &patterns, BTreeMap::new())
    }

    fn name_of(r: &NetclassResolver, net: &str) -> u16 {
        r.resolve(net).class.0
    }

    #[test]
    fn plain_patterns_match_whole_names_only() {
        let r = resolver(&[("CAM", 1), ("IO_*", 2)]);
        assert_eq!(name_of(&r, "CAM"), 1);
        assert_eq!(name_of(&r, "CAM_FEED"), 0);
        assert_eq!(name_of(&r, "X_CAM"), 0);
        assert_eq!(name_of(&r, "IO_7"), 2);
    }

    #[test]
    fn regex_patterns_are_anchored() {
        let r = resolver(&[("ADC[0-9]+", 1), ("Net-(R1-Pad1)", 2)]);
        assert_eq!(name_of(&r, "ADC12"), 1);
        assert_eq!(name_of(&r, "XADC1"), 0);
        // Parentheses are a group to the regex, literal to the wildcard.
        assert_eq!(name_of(&r, "Net-(R1-Pad1)"), 2);
    }

    #[test]
    fn escaped_names_fall_back_to_their_bus_member() {
        // KiCad parses `{slash}` as a one-member bus group.
        let r = resolver(&[("slash", 2)]);
        assert_eq!(name_of(&r, "Net-(U9-A{slash}B)"), 2);
        assert_eq!(name_of(&r, "unconnected-(P3-X{slash}Y-Pad2)"), 2);
        // Not a group: a space before the brace.
        assert_eq!(name_of(&r, "/A B{slash}C"), 0);
        // Formatting braces are not a group.
        assert_eq!(name_of(&r, "V_{slash}"), 0);
        assert_eq!(name_of(&r, "slash_net"), 0);
    }

    #[test]
    fn a_bus_needs_every_member_in_one_class() {
        let r = resolver(&[("D0", 1), ("D1", 1), ("D2", 2)]);
        assert_eq!(name_of(&r, "D[0..1]"), 1);
        assert_eq!(name_of(&r, "D[0..2]"), 0);
        assert_eq!(name_of(&r, "X{D0 D1}"), 1);
    }

    #[test]
    fn bus_patterns_expand() {
        let r = resolver(&[("D[0..3]", 1), ("G{P Q}", 2)]);
        assert_eq!(name_of(&r, "D2"), 1);
        assert_eq!(name_of(&r, "D4"), 0);
        assert_eq!(name_of(&r, "Q"), 2);
    }

    #[test]
    fn several_matches_take_the_highest_precedence_and_report_it() {
        let r = resolver(&[("IO_*", 2), ("IO_PWR", 1)]);
        let got = r.resolve("IO_PWR");
        // Equal (absent) priorities fall back to file order, then name.
        assert_eq!(got.class, ClassId(1));
        assert!(got.ambiguous);
        let classes = [class("Default"), class("A"), class("B")];
        let pats = vec![
            ("IO_*".to_string(), ClassId(2)),
            ("IO_PWR".to_string(), ClassId(1)),
        ];
        let r = NetclassResolver::new(&classes, &[None, Some(5), Some(1)], &pats, BTreeMap::new());
        assert_eq!(r.resolve("IO_PWR").class, ClassId(2));
    }

    #[test]
    fn explicit_assignments_win() {
        let classes = [class("Default"), class("A"), class("B")];
        let pats = vec![("N".to_string(), ClassId(1))];
        let explicit = BTreeMap::from([("N".to_string(), vec![ClassId(2)])]);
        let r = NetclassResolver::new(&classes, &[None, None, None], &pats, explicit);
        assert_eq!(r.resolve("N").class, ClassId(2));
    }

    fn membership(patterns: &[(&str, &str)], complete: &[&str]) -> NetclassMembership {
        let patterns: Vec<(String, String)> = patterns
            .iter()
            .map(|(p, c)| (p.to_string(), c.to_string()))
            .collect();
        NetclassMembership::new(
            &patterns,
            BTreeMap::new(),
            complete.iter().map(|c| c.to_string()).collect(),
        )
    }

    #[test]
    fn membership_follows_a_slash_name_to_its_member() {
        // A class leaving fields to the Default: the member gets a
        // composite, which the one-member group still takes.
        let m = membership(&[("USB_*", "USB"), ("slash", "USB")], &[]);
        let got = m.of("Net-(U8001-EN{slash}IN1)");
        assert_eq!(got.classes, vec!["USB".to_string()]);
        assert_eq!(got.via_bus_members, vec!["slash".to_string()]);
        assert_eq!(got.entries, vec![1]);
        // Without the pattern the net is in Default alone.
        let m = membership(&[("USB_*", "USB")], &[]);
        assert_eq!(m.of("Net-(U8001-EN{slash}IN1)"), Membership::default());
        // Direct matches name no members.
        let got = m.of("USB_DP");
        assert_eq!((got.classes.len(), got.via_bus_members.len()), (1, 0));
    }

    #[test]
    fn a_bus_shares_a_class_only_through_one_complete_class() {
        let pats = [("D0", "A"), ("D1", "A")];
        // Each member gets its own composite: no shared class.
        assert!(membership(&pats, &[]).of("D[0..1]").classes.is_empty());
        // A complete class is one object for both members.
        let got = membership(&pats, &["A"]).of("D[0..1]");
        assert_eq!(got.classes, vec!["A".to_string()]);
        assert_eq!(
            got.via_bus_members,
            vec!["D0".to_string(), "D1".to_string()]
        );
        // Two classes on one member make a composite even when complete.
        let got = membership(&[("D0", "A"), ("D0", "B"), ("D1", "A")], &["A", "B"]).of("D[0..1]");
        assert!(got.classes.is_empty());
    }

    #[test]
    fn membership_combines_explicit_assignments_and_patterns() {
        let m = NetclassMembership::new(
            &[("N".to_string(), "A".to_string())],
            BTreeMap::from([("N".to_string(), vec!["B".to_string()])]),
            Default::default(),
        );
        let got = m.of("N");
        assert_eq!(got.classes, vec!["A".to_string(), "B".to_string()]);
        assert_eq!(got.entries, vec![0]);
    }

    #[test]
    fn membership_matches_anchored_regex_and_wildcards() {
        let m = membership(
            &[("ADC[0-9]+", "A"), ("HV_*", "B"), ("Net-(R1-Pad1)", "C")],
            &[],
        );
        assert_eq!(m.of("ADC12").classes, vec!["A".to_string()]);
        assert!(m.of("XADC1").classes.is_empty());
        assert_eq!(m.of("HV_IN").classes, vec!["B".to_string()]);
        assert!(m.of("LV_IN").classes.is_empty());
        assert_eq!(m.of("Net-(R1-Pad1)").classes, vec!["C".to_string()]);
    }

    #[test]
    fn bus_parsing() {
        assert_eq!(
            bus_members("D[0..2]"),
            Some(vec!["D0".into(), "D1".into(), "D2".into()])
        );
        assert_eq!(
            bus_members("A[08..10]"),
            Some(vec!["A08".into(), "A09".into(), "A10".into()])
        );
        assert_eq!(bus_members("D[3..3]"), None);
        assert_eq!(bus_members("D[0..3]x"), None);
        assert_eq!(
            bus_members("USB{DP DM}"),
            Some(vec!["DP".into(), "DM".into()])
        );
        assert_eq!(
            bus_members("G{A[0..1],B}"),
            Some(vec!["A0".into(), "A1".into(), "B".into()])
        );
        assert_eq!(bus_members("PLAIN"), None);
        assert_eq!(bus_members("open{A"), None);
    }
}
