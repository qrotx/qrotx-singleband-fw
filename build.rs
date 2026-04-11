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
    // 2. CMSIS-DSP filtering functions.
    //
    //    Add the submodule first:
    //      git submodule add https://github.com/ARM-software/CMSIS-DSP c_src/CMSIS-DSP
    // -----------------------------------------------------------------------
    cc::Build::new()
        .flag("-mcpu=cortex-m4")
        .flag("-mfpu=fpv4-sp-d16")
        .flag("-mfloat-abi=hard")
        .flag("-mthumb")
        .flag("-O2")
        .flag("-w")                             // suppress third-party warnings
        .define("ARM_MATH_CM4",      None)
        .define("__FPU_PRESENT",     "1")
        .define("ARM_MATH_LOOPUNROLL", None)    // enable 4× loop unrolling
        .define("__GNUC_PYTHON__",   None)      // skip cmsis_compiler.h (CMSIS-Core not submodule'd)
        .include("c_src/CMSIS-DSP/Include")
        .include("c_src/CMSIS-DSP/PrivateInclude")
        .file("c_src/CMSIS-DSP/Source/FilteringFunctions/arm_fir_decimate_q31.c")
        .file("c_src/CMSIS-DSP/Source/FilteringFunctions/arm_fir_decimate_init_q31.c")
        .file("c_src/CMSIS-DSP/Source/FilteringFunctions/arm_fir_interpolate_q31.c")
        .file("c_src/CMSIS-DSP/Source/FilteringFunctions/arm_fir_interpolate_init_q31.c")
        .file("c_src/CMSIS-DSP/Source/FilteringFunctions/arm_biquad_cascade_df2T_f32.c")
        .file("c_src/CMSIS-DSP/Source/FilteringFunctions/arm_biquad_cascade_df2T_init_f32.c")
        .compile("cmsis_dsp");

    println!("cargo:rerun-if-changed=c_src/CMSIS-DSP");

    // -----------------------------------------------------------------------
    // 3. Custom SSB filter (based on CMSIS-DSP) — add file before enabling.
    //    Place your implementation at c_src/ssb_filter.c (header: ssb_filter.h).
    // -----------------------------------------------------------------------
    // cc::Build::new()
    //     .flag("-mcpu=cortex-m4")
    //     .flag("-mfpu=fpv4-sp-d16")
    //     .flag("-mfloat-abi=hard")
    //     .flag("-mthumb")
    //     .flag("-O2")
    //     .define("ARM_MATH_CM4", None)
    //     .define("__FPU_PRESENT", "1")
    //     .define("__GNUC_PYTHON__", None)
    //     .include("c_src/CMSIS-DSP/Include")
    //     .include("c_src")
    //     .file("c_src/ssb_filter.c")
    //     .compile("ssb_filter");
    //
    // println!("cargo:rerun-if-changed=c_src/ssb_filter.c");
    // println!("cargo:rerun-if-changed=c_src/ssb_filter.h");

    // Re-run this script only when these files change.
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=memory.x");
}
