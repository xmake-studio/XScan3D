//! Dev tool: exports real captures in the viewer's wire format, so the UI can
//! be developed in a plain browser against /dev-assets (see src/lib/mock.ts).
//! cargo run --release --example dev_assets -- <out_dir> <scan.bin>...
use xscan3d_lib::{capture::Capture, clouds, geometry, jobs, meshing, util};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let out = std::path::Path::new(&a[1]);
    std::fs::create_dir_all(out).unwrap();
    let m = geometry::Mount::default();
    let mut entries = Vec::new();
    for (k, path) in a[2..].iter().enumerate() {
        let p = std::path::Path::new(path);
        let id = p.file_stem().unwrap().to_string_lossy().to_string();
        let cap = Capture::from_bytes(&std::fs::read(p).unwrap());
        let coef = geometry::fit_microstep(&cap, &m);
        let cloud = geometry::build_cloud(&cap, &m, coef.as_deref()).unwrap();
        let sweep = cap.config.map(|c| c.degrees).unwrap_or(90.0);
        let n = cloud.len();
        let e = clouds::CloudEntry::new(id.clone(), 1, cloud.xyz, cloud.dist, cloud.platform, None, sweep);
        std::fs::write(out.join(format!("{id}.xsc")), clouds::serialize(&e)).unwrap();
        let created = std::fs::metadata(p)
            .and_then(|m| m.modified())
            .map(|t| chrono::DateTime::<chrono::Local>::from(t).to_rfc3339())
            .unwrap();
        entries.push(serde_json::json!({ "id": id, "points": n, "created": created, "duration": cap.sweep_seconds() }));
        println!("{id}: {n} points");
        if k == 0 {
            let pts = util::to_f64(&e.xyz);
            let scan = vec![0u16; pts.len()];
            let prm = meshing::MeshParams::preset("medium", 3.0, 2_500_000, 0);
            let (mesh, info) = meshing::reconstruct(&pts, &scan, &[[0.0; 3]], &prm, &|_| {}, &|| false).unwrap();
            std::fs::write(out.join("mesh.xsm"), jobs::mesh_bytes(&mesh)).unwrap();
            println!("mesh: {} tris", info.tris);
        }
    }
    std::fs::write(out.join("manifest.json"), serde_json::to_vec_pretty(&serde_json::json!({ "scans": entries })).unwrap()).unwrap();
}
