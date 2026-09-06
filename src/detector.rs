use std::collections::HashMap;

use crate::Detection;

pub fn merge_nearby_detections(mut detections: Vec<Detection>, margin: u32) -> Vec<Detection> {
    let mut changed = true;

    while changed {
        changed = false;

        'outer: for i in 0..detections.len() {
            for j in (i + 1)..detections.len() {
                if boxes_near(&detections[i], &detections[j], margin) {
                    let b = detections.remove(j);
                    merge_detection(&mut detections[i], &b);

                    changed = true;
                    break 'outer;
                }
            }
        }
    }

    detections.sort_by(|a, b| b.area.cmp(&a.area));
    detections
}

fn boxes_near(a: &Detection, b: &Detection, margin: u32) -> bool {
    let ax0 = a.x.saturating_sub(margin);
    let ay0 = a.y.saturating_sub(margin);
    let ax1 = a.x.saturating_add(a.width).saturating_add(margin);
    let ay1 = a.y.saturating_add(a.height).saturating_add(margin);

    let bx0 = b.x;
    let by0 = b.y;
    let bx1 = b.x.saturating_add(b.width);
    let by1 = b.y.saturating_add(b.height);

    bx0 < ax1 && bx1 > ax0 && by0 < ay1 && by1 > ay0
}

fn merge_detection(a: &mut Detection, b: &Detection) {
    let area_a = a.area as u64;
    let area_b = b.area as u64;
    let total_area = area_a + area_b;

    let x0 = a.x.min(b.x);
    let y0 = a.y.min(b.y);

    let x1 = (a.x + a.width).max(b.x + b.width);
    let y1 = (a.y + a.height).max(b.y + b.height);

    a.center_x = ((a.center_x as u64 * area_a + b.center_x as u64 * area_b) / total_area) as u32;

    a.center_y = ((a.center_y as u64 * area_a + b.center_y as u64 * area_b) / total_area) as u32;

    a.x = x0;
    a.y = y0;
    a.width = x1 - x0;
    a.height = y1 - y0;
    a.area = total_area as usize;
    a.max_temp_raw = a.max_temp_raw.max(b.max_temp_raw);
    a.max_temp_c = None;
}

pub fn dilate3x3(src: &[bool], width: u32, height: u32) -> Vec<bool> {
    let w = width as usize;
    let h = height as usize;
    let mut out = vec![false; src.len()];

    for y in 0..h {
        for x in 0..w {
            let mut on = false;

            for dy in -1isize..=1 {
                let yy = y as isize + dy;
                if yy < 0 || yy >= h as isize {
                    continue;
                }

                for dx in -1isize..=1 {
                    let xx = x as isize + dx;
                    if xx < 0 || xx >= w as isize {
                        continue;
                    }

                    if src[yy as usize * w + xx as usize] {
                        on = true;
                        break;
                    }
                }

                if on {
                    break;
                }
            }

            out[y * w + x] = on;
        }
    }

    out
}

pub fn erode3x3(src: &[bool], width: u32, height: u32) -> Vec<bool> {
    let w = width as usize;
    let h = height as usize;
    let mut out = vec![false; src.len()];

    for y in 0..h {
        for x in 0..w {
            let mut on = true;

            for dy in -1isize..=1 {
                let yy = y as isize + dy;
                if yy < 0 || yy >= h as isize {
                    on = false;
                    break;
                }

                for dx in -1isize..=1 {
                    let xx = x as isize + dx;
                    if xx < 0 || xx >= w as isize {
                        on = false;
                        break;
                    }

                    if !src[yy as usize * w + xx as usize] {
                        on = false;
                        break;
                    }
                }

                if !on {
                    break;
                }
            }

            out[y * w + x] = on;
        }
    }

    out
}

// blob detection (connected components)
#[derive(Default)]
struct Blob {
    min_x: u32,
    max_x: u32,
    min_y: u32,
    max_y: u32,
    area: usize,
    sum_x: u64,
    sum_y: u64,
    max_val: u16,
}

impl Blob {
    fn add(&mut self, x: u32, y: u32, val: u16) {
        if self.area == 0 {
            self.min_x = x;
            self.max_x = x;
            self.min_y = y;
            self.max_y = y;
            self.area = 1;
            self.sum_x = x as u64;
            self.sum_y = y as u64;
            self.max_val = val;
            return;
        }

        self.min_x = self.min_x.min(x);
        self.max_x = self.max_x.max(x);
        self.min_y = self.min_y.min(y);
        self.max_y = self.max_y.max(y);
        self.area += 1;
        self.sum_x += x as u64;
        self.sum_y += y as u64;
        self.max_val = self.max_val.max(val);
    }
}

struct UnionFind {
    parent: Vec<usize>,
    rank: Vec<u8>,
}

impl UnionFind {
    fn new() -> Self {
        Self {
            parent: vec![0],
            rank: vec![0],
        }
    }

    fn add(&mut self) -> usize {
        let id = self.parent.len();
        self.parent.push(id);
        self.rank.push(0);
        id
    }

    fn find(&mut self, item: usize) -> usize {
        if self.parent[item] != item {
            self.parent[item] = self.find(self.parent[item]);
        }
        self.parent[item]
    }

    fn union(&mut self, a: usize, b: usize) -> usize {
        let root_a = self.find(a);
        let root_b = self.find(b);

        if root_a == root_b {
            return root_a;
        }

        if self.rank[root_a] < self.rank[root_b] {
            self.parent[root_a] = root_b;
            root_b
        } else if self.rank[root_a] > self.rank[root_b] {
            self.parent[root_b] = root_a;
            root_a
        } else {
            self.parent[root_b] = root_a;
            self.rank[root_a] += 1;
            root_a
        }
    }
}

pub fn find_blobs(
    mask: &[bool],
    values: &[u16],
    width: u32,
    height: u32,
    min_area: usize,
    roi: &Option<(u32, u32, u32, u32)>,
) -> Vec<Detection> {
    debug_assert_eq!(mask.len(), values.len());

    let w = width as usize;
    let h = height as usize;

    let mut labels = vec![0usize; mask.len()];
    let mut uf = UnionFind::new();

    for y in 0..h {
        for x in 0..w {
            let idx = y * w + x;

            if !mask[idx] {
                continue;
            }

            let neighbors = [
                if x > 0 {
                    labels[idx - 1] // left
                } else {
                    0
                },
                if x > 0 && y > 0 {
                    labels[idx - w - 1] // up-left
                } else {
                    0
                },
                if y > 0 {
                    labels[idx - w] // up
                } else {
                    0
                },
                if x + 1 < w && y > 0 {
                    labels[idx - w + 1] // up-right
                } else {
                    0
                },
            ];

            let mut label = 0usize;

            for neighbor in neighbors {
                if neighbor == 0 {
                    continue;
                }

                label = if label == 0 {
                    uf.find(neighbor)
                } else {
                    uf.union(label, neighbor)
                };
            }

            labels[idx] = if label == 0 { uf.add() } else { label };
        }
    }

    let mut blobs: HashMap<usize, Blob> = HashMap::new();

    for y in 0..h {
        for x in 0..w {
            let idx = y * w + x;

            if !mask[idx] {
                continue;
            }

            let label = uf.find(labels[idx]);
            blobs
                .entry(label)
                .or_default()
                .add(x as u32, y as u32, values[idx]);
        }
    }

    let mut detections: Vec<Detection> = blobs
        .into_iter()
        .filter(|(_, blob)| blob.area >= min_area)
        .filter(|(_, blob)| {
            let Some((rx, ry, rw, rh)) = roi else {
                return true;
            };

            blob.max_x >= *rx
                && blob.min_x < *rx + *rw
                && blob.max_y >= *ry
                && blob.min_y < *ry + *rh
        })
        .map(|(id, blob)| Detection {
            id,
            x: blob.min_x,
            y: blob.min_y,
            width: blob.max_x - blob.min_x + 1,
            height: blob.max_y - blob.min_y + 1,
            area: blob.area,
            center_x: (blob.sum_x / blob.area as u64) as u32,
            center_y: (blob.sum_y / blob.area as u64) as u32,
            max_temp_raw: blob.max_val,
            max_temp_c: None,
        })
        .collect();

    detections.sort_by(|a, b| b.area.cmp(&a.area));

    detections
}
