use crate::Detection;

// tracker (stable IDs across frames)
pub struct Track {
    id: u64,

    cx: f32,
    cy: f32,

    x: u32,
    y: u32,
    width: u32,
    height: u32,

    last_seen: u64,
    hits: u32,
}

pub struct Tracker {
    tracks: Vec<Track>,
    next_id: u64,
    max_dist_sq: f32,
    max_missing: u64,
}

impl Tracker {
    pub fn new(max_dist: f32, max_missing: u64) -> Self {
        Self {
            tracks: Vec::new(),
            next_id: 1,
            max_dist_sq: max_dist * max_dist,
            max_missing,
        }
    }

    fn boxes_overlap_with_margin(track: &Track, det: &Detection, margin: u32) -> bool {
        let tx0 = track.x.saturating_sub(margin);
        let ty0 = track.y.saturating_sub(margin);

        let tx1 = track.x.saturating_add(track.width).saturating_add(margin);

        let ty1 = track.y.saturating_add(track.height).saturating_add(margin);

        let dx0 = det.x;
        let dy0 = det.y;
        let dx1 = det.x.saturating_add(det.width);
        let dy1 = det.y.saturating_add(det.height);

        dx0 < tx1 && dx1 > tx0 && dy0 < ty1 && dy1 > ty0
    }

    pub fn update(&mut self, detections: &mut [Detection], frame_no: u64) {
        for det in detections.iter_mut() {
            let dx = det.center_x as f32;
            let dy = det.center_y as f32;

            let mut best: Option<(usize, f32)> = None;

            for (i, track) in self.tracks.iter().enumerate() {
                // track may be assigned to only one detection per frame
                if track.last_seen == frame_no {
                    continue;
                }

                let dist_sq = (track.cx - dx).powi(2) + (track.cy - dy).powi(2);

                // normal association by centroid distance
                //
                // additionally keep association when the new blob still
                // overlaps the previous bbox with a small margin. This
                // helps when one physical object's shape changes quickly
                // and its centroid jumps
                let spatially_close =
                    dist_sq < self.max_dist_sq || Self::boxes_overlap_with_margin(track, det, 8);

                if !spatially_close {
                    continue;
                }

                if best.map_or(true, |(_, best_dist)| dist_sq < best_dist) {
                    best = Some((i, dist_sq));
                }
            }

            match best {
                Some((i, _)) => {
                    let track = &mut self.tracks[i];

                    track.cx = track.cx * 0.5 + dx * 0.5;
                    track.cy = track.cy * 0.5 + dy * 0.5;

                    track.x = det.x;
                    track.y = det.y;
                    track.width = det.width;
                    track.height = det.height;

                    track.last_seen = frame_no;
                    track.hits += 1;

                    det.id = track.id as usize;
                }

                None => {
                    det.id = self.next_id as usize;

                    self.tracks.push(Track {
                        id: self.next_id,

                        cx: dx,
                        cy: dy,

                        x: det.x,
                        y: det.y,
                        width: det.width,
                        height: det.height,

                        last_seen: frame_no,
                        hits: 1,
                    });

                    self.next_id += 1;
                }
            }
        }

        self.tracks
            .retain(|track| frame_no - track.last_seen <= self.max_missing);
    }

    pub fn is_confirmed(&self, id: usize) -> bool {
        self.tracks
            .iter()
            .any(|track| track.id as usize == id && track.hits >= 2)
    }
}
