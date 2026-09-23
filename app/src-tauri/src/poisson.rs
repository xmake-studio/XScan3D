//! FFI to the screened Poisson solver in cpp/poisson_bridge.cpp.

use std::ffi::CStr;
use std::os::raw::{c_char, c_float, c_int};

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PrParams {
    pub depth: c_int,
    pub full_depth: c_int,
    pub samples_per_node: c_float,
    pub point_weight: c_float,
    pub scale: c_float,
    pub iters: c_int,
    pub threads: c_int,
    pub linear_fit: c_int,
}

#[repr(C)]
struct PrResult {
    verts: *mut f32,
    density: *mut f32,
    tris: *mut u32,
    n_verts: usize,
    n_tris: usize,
    error: [c_char; 512],
}

extern "C" {
    fn pr_reconstruct(
        points: *const f32,
        normals: *const f32,
        count: usize,
        params: *const PrParams,
        out: *mut PrResult,
    ) -> c_int;
    fn pr_free(result: *mut PrResult);
}

/// Raw solver output: vertices, their sampling density, and triangles.
pub struct PoissonMesh {
    pub verts: Vec<[f32; 3]>,
    pub density: Vec<f32>,
    pub tris: Vec<[u32; 3]>,
}

/// Runs the solver over oriented points. `points` and `normals` are parallel.
pub fn reconstruct(
    points: &[[f32; 3]],
    normals: &[[f32; 3]],
    params: &PrParams,
) -> Result<PoissonMesh, String> {
    assert_eq!(points.len(), normals.len());
    let mut out = PrResult {
        verts: std::ptr::null_mut(),
        density: std::ptr::null_mut(),
        tris: std::ptr::null_mut(),
        n_verts: 0,
        n_tris: 0,
        error: [0; 512],
    };
    let rc = unsafe {
        pr_reconstruct(
            points.as_ptr() as *const f32,
            normals.as_ptr() as *const f32,
            points.len(),
            params,
            &mut out,
        )
    };
    if rc != 0 {
        let msg = unsafe { CStr::from_ptr(out.error.as_ptr()) }
            .to_string_lossy()
            .into_owned();
        unsafe { pr_free(&mut out) };
        return Err(if msg.is_empty() { "reconstruction failed".into() } else { msg });
    }
    let result = unsafe {
        let verts = if out.n_verts > 0 {
            std::slice::from_raw_parts(out.verts as *const [f32; 3], out.n_verts).to_vec()
        } else {
            Vec::new()
        };
        let density = if out.n_verts > 0 {
            std::slice::from_raw_parts(out.density, out.n_verts).to_vec()
        } else {
            Vec::new()
        };
        let tris = if out.n_tris > 0 {
            std::slice::from_raw_parts(out.tris as *const [u32; 3], out.n_tris).to_vec()
        } else {
            Vec::new()
        };
        PoissonMesh { verts, density, tris }
    };
    unsafe { pr_free(&mut out) };
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sphere_reconstructs() {
        // Fibonacci sphere of radius 100 with outward normals.
        let n = 20000;
        let mut pts = Vec::with_capacity(n);
        let mut nrm = Vec::with_capacity(n);
        let golden = std::f32::consts::PI * (3.0 - 5f32.sqrt());
        for i in 0..n {
            let y = 1.0 - 2.0 * (i as f32 + 0.5) / n as f32;
            let r = (1.0 - y * y).sqrt();
            let th = golden * i as f32;
            let d = [r * th.cos(), y, r * th.sin()];
            pts.push([d[0] * 100.0, d[1] * 100.0, d[2] * 100.0]);
            nrm.push(d);
        }
        let params = PrParams {
            depth: 7,
            full_depth: 5,
            samples_per_node: 1.5,
            point_weight: 2.0,
            scale: 1.1,
            iters: 8,
            threads: 0,
            linear_fit: 0,
        };
        let m = reconstruct(&pts, &nrm, &params).expect("poisson");
        assert!(m.tris.len() > 1000, "tris {}", m.tris.len());
        let mean_r: f32 = m
            .verts
            .iter()
            .map(|v| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt())
            .sum::<f32>()
            / m.verts.len() as f32;
        assert!((mean_r - 100.0).abs() < 3.0, "mean radius {mean_r}");
    }
}
