fn main() {
    // Kazhdan's screened Poisson solver, compiled into the binary. Always
    // optimised: an unoptimised build of this template-heavy solver is two
    // orders of magnitude slower, which makes even debug runs unusable.
    println!("cargo:rerun-if-changed=cpp/poisson_bridge.cpp");
    println!("cargo:rerun-if-changed=cpp/poisson_bridge.h");
    let mut build = cc::Build::new();
    build
        .cpp(true)
        .file("cpp/poisson_bridge.cpp")
        .include("cpp")
        .include("third_party/PoissonRecon/Src")
        .opt_level(2)
        .debug(false)
        .define("NDEBUG", None)
        .warnings(false);
    if build.get_compiler().is_like_msvc() {
        // /MT to match the +crt-static Rust side (see .cargo/config.toml), so
        // the exe needs no Visual C++ redistributable.
        build
            .static_crt(true)
            .flag("/std:c++17")
            .flag("/EHsc")
            .flag("/bigobj")
            .flag("/utf-8")
            .flag("/permissive-")
            .flag("/Zc:__cplusplus")
            .define("_CRT_SECURE_NO_WARNINGS", None)
            .define("NOMINMAX", None);
    } else {
        build.flag("-std=c++17").flag("-pthread");
    }
    build.compile("poisson_bridge");

    tauri_build::build()
}
