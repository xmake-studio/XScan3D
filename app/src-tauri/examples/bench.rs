//! Dev tool: exercise calibration, registration and meshing on real captures.
//! cargo run --release --example bench -- calib <bin>
//! cargo run --release --example bench -- reg <src.bin> <tgt.bin>
//! cargo run --release --example bench -- mesh <bin> <quality> [out.ply]
use std::time::Instant;
use xscan3d_lib::{calibration, capture::Capture, export, geometry, meshing, registration, util};

fn load(path: &str) -> (Capture, Vec<[f64; 3]>) {
    let cap = Capture::from_bytes(&std::fs::read(path).unwrap());
    let m = geometry::Mount::default();
    let coef = geometry::fit_microstep(&cap, &m);
    let c = geometry::build_cloud(&cap, &m, coef.as_deref()).unwrap();
    let p = util::to_f64(&c.xyz);
    (cap, p)
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    match a[1].as_str() {
        "calib" => {
            let (cap, _) = load(&a[2]);
            let t = Instant::now();
            let base = geometry::Mount::default();
            let r = calibration::calibrate(&cap, &base, &|s, f| eprint!("\r{:?} {:.0}%   ", s, f * 100.0), &|| false);
            eprintln!();
            println!("{:.1}s {:#?}", t.elapsed().as_secs_f64(), r.map_err(|e| e.to_string()));
        }
        "range" => {
            let (cap, _) = load(&a[2]);
            let t = Instant::now();
            let base = geometry::Mount::default();
            let coef = geometry::fit_microstep(&cap, &base);
            let x = [base.lidar_rotation, base.emitter_spacing, base.scan_tilt];
            let r = calibration::fit_range_error(&cap, &base, &x, coef.as_deref(), &|| false).unwrap();
            println!("{:.1}s {:?}", t.elapsed().as_secs_f64(), r);
            println!("at 3m: {:.2} mm", geometry::range_error_at(r.as_deref(), 3000.0));
        }
        "reg" => {
            let (_, s) = load(&a[2]);
            let (_, t) = load(&a[3]);
            let t0 = Instant::now();
            let prm = registration::RegParams { structure: true, ..Default::default() };
            let r = registration::register(&s, &t, &prm, &|st, i, n| eprintln!("  {:?} {}/{}", st, i, n), &|| false).unwrap();
            println!("{:.1}s {:?}", t0.elapsed().as_secs_f64(), registration::verdict(&r));
            println!("overlap {:.3} rmse {:.2} stab {:.3} yaw {:.0}", r.overlap, r.rmse, r.stability, r.yaw);
            println!("T = {:?}", r.t);
        }
        "mesh" => {
            let (_, p) = load(&a[2]);
            let prm = meshing::MeshParams::preset(&a[3], 3.0, 2_500_000, 0);
            let t0 = Instant::now();
            let scan = vec![0u16; p.len()];
            let (m, info) = meshing::reconstruct(&p, &scan, &[[0.0; 3]], &prm, &|s| eprintln!("  {:?} {:.1}s", s, t0.elapsed().as_secs_f64()), &|| false).unwrap();
            println!("{:?}", info);
            if let Some(out) = a.get(4) {
                export::write_ply_mesh(std::path::Path::new(out), &m).unwrap();
            }
        }
        _ => {}
    }
}
