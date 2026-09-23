//! File formats: PLY point clouds and meshes, OBJ and STL meshes, and a PLY
//! reader tolerant of whatever MeshLab or CloudCompare wrote.

use std::fs::File;
use std::io::{self, BufRead, BufReader, BufWriter, Read, Write};
use std::path::Path;

use crate::meshing::Mesh;

/// The height ramp shared with the viewer (src/lib/render/palette.ts): deep
/// indigo through blue, teal and lime to warm amber.
pub const PALETTE: [[f32; 3]; 5] = [
    [0.231, 0.169, 0.710],
    [0.184, 0.486, 0.965],
    [0.122, 0.761, 0.761],
    [0.620, 0.878, 0.290],
    [1.000, 0.812, 0.247],
];

pub fn ramp(t: f32) -> [u8; 3] {
    let t = if t.is_finite() { t.clamp(0.0, 1.0) } else { 0.5 };
    let x = t * (PALETTE.len() - 1) as f32;
    let i = (x.floor() as usize).min(PALETTE.len() - 2);
    let f = x - i as f32;
    let a = PALETTE[i];
    let b = PALETTE[i + 1];
    let mut out = [0u8; 3];
    for k in 0..3 {
        out[k] = ((a[k] + (b[k] - a[k]) * f) * 255.0).round() as u8;
    }
    out
}

fn tmp_path(path: &Path) -> std::path::PathBuf {
    let mut p = path.as_os_str().to_owned();
    p.push(".part");
    p.into()
}

/// Writes to a temporary name and swaps it into place, so a failed export
/// never leaves half a file where a good one was.
fn atomic_write(path: &Path, f: impl FnOnce(&mut BufWriter<File>) -> io::Result<()>) -> io::Result<()> {
    let tmp = tmp_path(path);
    {
        let mut w = BufWriter::with_capacity(1 << 20, File::create(&tmp)?);
        f(&mut w)?;
        w.flush()?;
    }
    std::fs::rename(&tmp, path).inspect_err(|_| {
        let _ = std::fs::remove_file(&tmp);
    })
}

/// Binary PLY point cloud: float xyz, uchar rgb and the range as a scalar.
pub fn write_ply_points(path: &Path, xyz: &[[f32; 3]], rgb: &[[u8; 3]], range: Option<&[f32]>) -> io::Result<()> {
    atomic_write(path, |w| {
        write!(w, "ply\nformat binary_little_endian 1.0\ncomment XScan3D point cloud, millimetres\n")?;
        write!(w, "element vertex {}\n", xyz.len())?;
        w.write_all(b"property float x\nproperty float y\nproperty float z\n")?;
        w.write_all(b"property uchar red\nproperty uchar green\nproperty uchar blue\n")?;
        if range.is_some() {
            w.write_all(b"property float scalar_range\n")?;
        }
        w.write_all(b"end_header\n")?;
        let mut rec = Vec::with_capacity(19 * 65536);
        for (i, p) in xyz.iter().enumerate() {
            for v in p {
                rec.extend_from_slice(&v.to_le_bytes());
            }
            rec.extend_from_slice(&rgb[i]);
            if let Some(r) = range {
                rec.extend_from_slice(&r[i].to_le_bytes());
            }
            if rec.len() >= 19 * 65000 {
                w.write_all(&rec)?;
                rec.clear();
            }
        }
        w.write_all(&rec)
    })
}

/// Binary PLY mesh with per-vertex normals.
pub fn write_ply_mesh(path: &Path, m: &Mesh) -> io::Result<()> {
    atomic_write(path, |w| {
        write!(w, "ply\nformat binary_little_endian 1.0\ncomment XScan3D surface, millimetres\n")?;
        write!(w, "element vertex {}\n", m.verts.len())?;
        w.write_all(b"property float x\nproperty float y\nproperty float z\n")?;
        w.write_all(b"property float nx\nproperty float ny\nproperty float nz\n")?;
        write!(w, "element face {}\n", m.tris.len())?;
        w.write_all(b"property list uchar int vertex_indices\nend_header\n")?;
        let mut buf = Vec::with_capacity(1 << 20);
        for (v, n) in m.verts.iter().zip(&m.normals) {
            for x in v.iter().chain(n.iter()) {
                buf.extend_from_slice(&x.to_le_bytes());
            }
            if buf.len() > (1 << 20) - 64 {
                w.write_all(&buf)?;
                buf.clear();
            }
        }
        for t in &m.tris {
            buf.push(3);
            for &i in t {
                buf.extend_from_slice(&(i as i32).to_le_bytes());
            }
            if buf.len() > (1 << 20) - 64 {
                w.write_all(&buf)?;
                buf.clear();
            }
        }
        w.write_all(&buf)
    })
}

/// Wavefront OBJ with normals. Text, so larger and slower, but universal.
pub fn write_obj(path: &Path, m: &Mesh) -> io::Result<()> {
    atomic_write(path, |w| {
        w.write_all(b"# XScan3D surface, millimetres\n")?;
        for v in &m.verts {
            writeln!(w, "v {} {} {}", v[0], v[1], v[2])?;
        }
        for n in &m.normals {
            writeln!(w, "vn {:.4} {:.4} {:.4}", n[0], n[1], n[2])?;
        }
        for t in &m.tris {
            let (a, b, c) = (t[0] + 1, t[1] + 1, t[2] + 1);
            writeln!(w, "f {a}//{a} {b}//{b} {c}//{c}")?;
        }
        Ok(())
    })
}

/// Binary STL, for slicers and CAD.
pub fn write_stl(path: &Path, m: &Mesh) -> io::Result<()> {
    atomic_write(path, |w| {
        let mut header = [0u8; 80];
        let h = b"XScan3D surface, millimetres";
        header[..h.len()].copy_from_slice(h);
        w.write_all(&header)?;
        w.write_all(&(m.tris.len() as u32).to_le_bytes())?;
        let mut rec = [0u8; 50];
        for t in &m.tris {
            let a = m.verts[t[0] as usize];
            let b = m.verts[t[1] as usize];
            let c = m.verts[t[2] as usize];
            let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
            let mut n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
            let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            if l > 0.0 {
                n = [n[0] / l, n[1] / l, n[2] / l];
            }
            let mut o = 0;
            for x in n.iter().chain(a.iter()).chain(b.iter()).chain(c.iter()) {
                rec[o..o + 4].copy_from_slice(&x.to_le_bytes());
                o += 4;
            }
            rec[48] = 0;
            rec[49] = 0;
            w.write_all(&rec)?;
        }
        Ok(())
    })
}

// --- reading -----------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
enum PlyType {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    F32,
    F64,
}

impl PlyType {
    fn parse(s: &str) -> Option<PlyType> {
        Some(match s {
            "char" | "int8" => PlyType::I8,
            "uchar" | "uint8" => PlyType::U8,
            "short" | "int16" => PlyType::I16,
            "ushort" | "uint16" => PlyType::U16,
            "int" | "int32" => PlyType::I32,
            "uint" | "uint32" => PlyType::U32,
            "float" | "float32" => PlyType::F32,
            "double" | "float64" => PlyType::F64,
            _ => return None,
        })
    }
    fn size(self) -> usize {
        match self {
            PlyType::I8 | PlyType::U8 => 1,
            PlyType::I16 | PlyType::U16 => 2,
            PlyType::I32 | PlyType::U32 | PlyType::F32 => 4,
            PlyType::F64 => 8,
        }
    }
    fn read(self, b: &[u8], big: bool) -> f64 {
        macro_rules! rd {
            ($t:ty, $n:expr) => {{
                let mut a = [0u8; $n];
                a.copy_from_slice(&b[..$n]);
                (if big { <$t>::from_be_bytes(a) } else { <$t>::from_le_bytes(a) }) as f64
            }};
        }
        match self {
            PlyType::I8 => b[0] as i8 as f64,
            PlyType::U8 => b[0] as f64,
            PlyType::I16 => rd!(i16, 2),
            PlyType::U16 => rd!(u16, 2),
            PlyType::I32 => rd!(i32, 4),
            PlyType::U32 => rd!(u32, 4),
            PlyType::F32 => rd!(f32, 4),
            PlyType::F64 => rd!(f64, 8),
        }
    }
}

enum Prop {
    Scalar(String, PlyType),
    List(PlyType, PlyType),
}

struct Element {
    name: String,
    count: usize,
    props: Vec<Prop>,
}

fn bad(msg: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg.into())
}

/// Reads a PLY's vertices: (xyz, rgb if present).
pub fn read_ply(path: &Path) -> io::Result<(Vec<[f32; 3]>, Option<Vec<[u8; 3]>>)> {
    let mut r = BufReader::with_capacity(1 << 20, File::open(path)?);
    let mut line = String::new();
    r.read_line(&mut line)?;
    if line.trim() != "ply" {
        return Err(bad("not a PLY file"));
    }
    let mut format = String::new();
    let mut elements: Vec<Element> = Vec::new();
    loop {
        line.clear();
        if r.read_line(&mut line)? == 0 {
            return Err(bad("header never ended"));
        }
        let tok: Vec<&str> = line.split_whitespace().collect();
        match tok.first().copied() {
            Some("format") => format = tok.get(1).unwrap_or(&"").to_string(),
            Some("element") => elements.push(Element {
                name: tok.get(1).unwrap_or(&"").to_string(),
                count: tok.get(2).and_then(|s| s.parse().ok()).unwrap_or(0),
                props: vec![],
            }),
            Some("property") => {
                let el = elements.last_mut().ok_or_else(|| bad("property before element"))?;
                if tok.get(1) == Some(&"list") {
                    let c = tok.get(2).and_then(|s| PlyType::parse(s)).ok_or_else(|| bad("bad list type"))?;
                    let t = tok.get(3).and_then(|s| PlyType::parse(s)).ok_or_else(|| bad("bad list type"))?;
                    el.props.push(Prop::List(c, t));
                } else {
                    let t = tok.get(1).and_then(|s| PlyType::parse(s)).ok_or_else(|| bad("unknown property type"))?;
                    el.props.push(Prop::Scalar(tok.get(2).unwrap_or(&"").to_string(), t));
                }
            }
            Some("end_header") => break,
            _ => {}
        }
    }
    let ascii = format == "ascii";
    let big = format == "binary_big_endian";
    if !ascii && !big && format != "binary_little_endian" {
        return Err(bad(format!("unsupported PLY format '{format}'")));
    }
    let mut xyz = Vec::new();
    let mut rgb: Option<Vec<[u8; 3]>> = None;
    let mut text = String::new();
    if ascii {
        r.read_to_string(&mut text)?;
    }
    let mut words = text.split_ascii_whitespace();
    for el in &elements {
        let is_vertex = el.name == "vertex";
        let col = |name: &str| {
            el.props.iter().position(|p| matches!(p, Prop::Scalar(n, _) if n == name))
        };
        let (ix, iy, iz) = (col("x"), col("y"), col("z"));
        let (ir, ig, ib) = (col("red"), col("green"), col("blue"));
        if is_vertex {
            if ix.is_none() || iy.is_none() || iz.is_none() {
                return Err(bad("vertex element has no x/y/z"));
            }
            xyz.reserve(el.count);
            if ir.is_some() && ig.is_some() && ib.is_some() {
                rgb = Some(Vec::with_capacity(el.count));
            }
        }
        let mut vals = vec![0.0f64; el.props.len()];
        let mut scratch = [0u8; 8];
        for _ in 0..el.count {
            for (k, p) in el.props.iter().enumerate() {
                match p {
                    Prop::Scalar(_, t) => {
                        vals[k] = if ascii {
                            words.next().and_then(|w| w.parse().ok()).ok_or_else(|| bad("truncated"))?
                        } else {
                            r.read_exact(&mut scratch[..t.size()])?;
                            t.read(&scratch, big)
                        };
                    }
                    Prop::List(c, t) => {
                        let n = if ascii {
                            words.next().and_then(|w| w.parse::<f64>().ok()).ok_or_else(|| bad("truncated"))? as usize
                        } else {
                            r.read_exact(&mut scratch[..c.size()])?;
                            c.read(&scratch, big) as usize
                        };
                        for _ in 0..n {
                            if ascii {
                                words.next();
                            } else {
                                r.read_exact(&mut scratch[..t.size()])?;
                            }
                        }
                    }
                }
            }
            if is_vertex {
                xyz.push([vals[ix.unwrap()] as f32, vals[iy.unwrap()] as f32, vals[iz.unwrap()] as f32]);
                if let Some(c) = rgb.as_mut() {
                    let conv = |v: f64, t: &Prop| -> u8 {
                        match t {
                            Prop::Scalar(_, PlyType::F32 | PlyType::F64) => (v * 255.0).clamp(0.0, 255.0) as u8,
                            _ => v.clamp(0.0, 255.0) as u8,
                        }
                    };
                    c.push([
                        conv(vals[ir.unwrap()], &el.props[ir.unwrap()]),
                        conv(vals[ig.unwrap()], &el.props[ig.unwrap()]),
                        conv(vals[ib.unwrap()], &el.props[ib.unwrap()]),
                    ]);
                }
            }
        }
        if is_vertex {
            break; // nothing after the vertices is needed
        }
    }
    Ok((xyz, rgb))
}
