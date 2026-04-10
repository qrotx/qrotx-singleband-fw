// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// build.rs — pre-build script
//   • Copies memory.x to OUT_DIR so the linker can find it.
//   • Provides a ready-made skeleton for linking C/assembler objects
//     (e.g. the ARM CMSIS-DSP library) when that becomes needed.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // -----------------------------------------------------------------------
    // 1. Publish memory.x into OUT_DIR for the cortex-m-rt linker script.
    // -----------------------------------------------------------------------
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());

    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();

    println!("cargo:rustc-link-search={}", out.display());

    // -----------------------------------------------------------------------
    // 2. C / assembler sources (uncomment when ARM DSP library is added).
    //
    //    Typical usage with CMSIS-DSP:
    //      • Clone / copy the CMSIS pack into c_src/
    //      • Uncomment the block below and adjust include paths.
    //      • Add `extern "C" { ... }` declarations in src/dsp.rs.
    //
    // cc::Build::new()
    //     .file("c_src/arm_dsp_wrapper.c")
    //     .flag("-mcpu=cortex-m4")
    //     .flag("-mfpu=fpv4-sp-d16")
    //     .flag("-mfloat-abi=hard")
    //     .flag("-mthumb")
    //     .flag("-O2")
    //     .flag("-DARM_MATH_CM4")
    //     .include("c_src/CMSIS/DSP/Include")
    //     .include("c_src/CMSIS/Core/Include")
    //     .compile("arm_dsp_wrapper");
    //
    // println!("cargo:rustc-link-lib=static=arm_dsp_wrapper");
    // -----------------------------------------------------------------------

    // Re-run this script only when these files change.
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=memory.x");
}
