//! Routed copper as negotiation keeps it: one record per committed piece
//! of a connection, with the courtyards each item touches so the rule
//! table can be asked the same question KiCad's DRC will ask.

use crate::api::{FootprintId, ItemKind, LayerId, NetId};
use crate::rules::RuleSubject;
use konnect_geom::{BBox, Circle, Coord, Point, Segment, Shape};

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct SegCopper {
    pub layer: LayerId,
    pub seg: Segment,
    pub courtyards: Vec<FootprintId>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ViaCopper {
    pub center: Point,
    pub diameter: Coord,
    pub drill: Coord,
    pub courtyards: Vec<FootprintId>,
}

impl ViaCopper {
    pub fn disc(&self) -> Circle {
        Circle::from_diameter(self.center, self.diameter)
    }

    pub fn hole(&self) -> Circle {
        Circle::from_diameter(self.center, self.drill)
    }
}

/// One piece of one connection, as committed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct PieceCopper {
    pub task: u32,
    pub piece: u8,
    pub net: NetId,
    pub segs: Vec<SegCopper>,
    pub vias: Vec<ViaCopper>,
    /// Union of every item's box; `None` for a piece with no copper (its
    /// terminals coincide).
    pub bbox: Option<BBox>,
}

impl PieceCopper {
    pub fn new(
        task: u32,
        piece: u8,
        net: NetId,
        segs: Vec<SegCopper>,
        vias: Vec<ViaCopper>,
    ) -> Self {
        let bbox = segs
            .iter()
            .map(|s| s.seg.bbox())
            .chain(vias.iter().map(|v| v.disc().bbox()))
            .reduce(BBox::union);
        PieceCopper {
            task,
            piece,
            net,
            segs,
            vias,
            bbox,
        }
    }

    /// Items of this piece present on `layer`: its segments there, and
    /// every (through) via.
    pub fn items_on(
        &self,
        layer: LayerId,
    ) -> impl Iterator<Item = (Shape, ItemKind, &[FootprintId])> {
        self.segs
            .iter()
            .filter(move |s| s.layer == layer)
            .map(|s| {
                (
                    Shape::Segment(s.seg),
                    ItemKind::Track,
                    s.courtyards.as_slice(),
                )
            })
            .chain(self.vias.iter().map(|v| {
                (
                    Shape::Circle(v.disc()),
                    ItemKind::Via,
                    v.courtyards.as_slice(),
                )
            }))
    }

    pub fn subject<'a>(&self, kind: ItemKind, courtyards: &'a [FootprintId]) -> RuleSubject<'a> {
        RuleSubject::new(kind, Some(self.net)).with_courtyards(courtyards)
    }
}
