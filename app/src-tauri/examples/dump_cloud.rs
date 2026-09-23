//! Dev tool: decode a .bin and dump the cloud for comparison with scan_proto.
//! cargo run --release --example dump_cloud -- <in.bin> <out_prefix> [auto]
use std::io::Write;
use xscan3d_lib::{capture::Capture, geometry};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let data = std::fs::read(&args[1]).expect("read");
    let t = std::time::Instant::now();
    let cap = Capture::from_bytes(&data);
    println!("samples {} telem {} parse {:.3}s", cap.len(), cap.telem.len(), t.elapsed().as_secs_f64());
    let m = geometry::Mount::default();
    let t = std::time::Instant::now();
    let coef = if args.get(3).map(|s| s == "auto").unwrap_or(false) {
        let c = geometry::fit_microstep(&cap, &m);
        println!("fit {:.3}s coef {:?}", t.elapsed().as_secs_f64(), c);
        c
    } else {
        None
    };
    let t = std::time::Instant::now();
    let cloud = geometry::build_cloud(&cap, &m, coef.as_deref()).expect("build");
    println!("points {} build {:.3}s", cloud.len(), t.elapsed().as_secs_f64());
    let mut f = std::fs::File::create(format!("{}_xyz.f32", args[2])).unwrap();
    for p in &cloud.xyz {
        for v in p {
            f.write_all(&v.to_le_bytes()).unwrap();
        }
    }
    if let Some(c) = coef {
        let mut f = std::fs::File::create(format!("{}_coef.txt", args[2])).unwrap();
        for v in c {
            writeln!(f, "{v}").unwrap();
        }
    }
}
