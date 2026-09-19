fn main() {
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("macos") {
        println!("cargo:rustc-link-arg-cdylib=-Wl,-install_name,@rpath/libposebridge_capi.dylib");
    }
    let crate_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let out = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let config = cbindgen::Config {
        language: cbindgen::Language::C,
        include_guard: Some("POSEBRIDGE_H".into()),
        documentation: true,
        cpp_compat: true,
        usize_is_size_t: true,
        ..Default::default()
    };
    cbindgen::Builder::new()
        .with_crate(crate_dir)
        .with_config(config)
        .generate()
        .expect("generate PoseBridge C header")
        .write_to_file(out.join("posebridge.h"));
    println!("cargo:rerun-if-changed=src/lib.rs");
}
