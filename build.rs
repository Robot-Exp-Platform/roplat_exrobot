fn main() {
    #[cfg(feature = "to_cxx")]
    {
        cxx_build::bridge("src/to_cxx.rs").compile("roplat_exrobot_cxx");

        println!("cargo:rerun-if-changed=src/to_cxx.rs");
    }
}
