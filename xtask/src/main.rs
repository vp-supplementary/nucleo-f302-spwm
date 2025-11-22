use std::io::Write;
use std::{
    env, fs,
    path::{Path, PathBuf},
    process::Command,
};

type DynError = Box<dyn std::error::Error>;

const CARGO: &str = env!("CARGO");

fn main() {
    if let Err(e) = try_main() {
        eprintln!("{e}");
        std::process::exit(-1);
    }
}

fn try_main() -> Result<(), DynError> {
    let task = env::args().nth(1);
    match task.as_deref() {
        Some("build") => build()?,
        Some("size") => size()?,
        Some("flash") => flash()?,
        Some("gdbserver") => gdbserver()?,
        _ => print_help(),
    }
    Ok(())
}

fn build() -> Result<(), DynError> {
    let app_name = "nucleo-f302-spwm";
    let app_dir = project_root().join(app_name);

    let status = Command::new(CARGO)
        .current_dir(app_dir)
        .args(["build"])
        .status()?;

    if !status.success() {
        Err("cargo build failed")?;
    }

    Ok(())
}

fn gdbserver() -> Result<(), DynError> {
    let platform = "thumbv7em-none-eabihf";
    let gdbserver_tool = "jlinkgdbservercl";
    let elf_output_dir = project_root().join("target").join(platform).join("debug");

    let status = Command::new(gdbserver_tool)
        .args(["-device", "STM32F302R8", "-if", "SWD", "-speed", "30000"])
        .current_dir(elf_output_dir)
        .status()?;

    if !status.success() {
        Err("cargo gdbserver failed")?;
    }

    Ok(())
}

fn size() -> Result<(), DynError> {
    let platform = "thumbv7em-none-eabihf";
    let size_tool = "llvm-size";
    let elf_output_dir = project_root().join("target").join(platform).join("debug");
    let elf_file = elf_output_dir.join("nucleo-f302-spwm");

    let status = Command::new(size_tool)
        .args(["-x", "-A", elf_file.to_str().unwrap()])
        .current_dir(elf_output_dir)
        .status()?;

    if !status.success() {
        Err("cargo size failed")?;
    }

    Ok(())
}

fn flash() -> Result<(), DynError> {
    let platform = "thumbv7em-none-eabihf";
    let objcopy = "llvm-objcopy";
    let elf_output_dir = project_root().join("target").join(platform).join("debug");
    let elf_file = elf_output_dir.join("nucleo-f302-spwm");
    let hex_file = elf_output_dir.join("nucleo-f302-spwm.hex");
    let flash_tool = "jlink";
    let cmd_flash_file = elf_output_dir.join("flash.jlink");
    let mut file = fs::File::create(cmd_flash_file.clone())?;

    let status = Command::new(objcopy)
        .args([
            "-O",
            "ihex",
            elf_file.to_str().unwrap(),
            hex_file.to_str().unwrap(),
        ])
        .current_dir(elf_output_dir.clone())
        .status()?;

    if !status.success() {
        Err("cargo flash failed")?;
    }

    writeln!(file, "r")?;
    writeln!(file, "loadfile {}", hex_file.to_str().unwrap())?;
    writeln!(file, "g")?;
    writeln!(file, "q")?;
    writeln!(file, "exit")?;
    drop(file);

    let status = Command::new(flash_tool)
        .args([
            "-If",
            "SWD",
            "-Speed",
            "30000",
            "-Device",
            "STM32F302R8",
            "-CommandFile",
            cmd_flash_file.to_str().unwrap(),
        ])
        .current_dir(elf_output_dir)
        .status()?;

    if !status.success() {
        Err("cargo flash failed")?;
    }

    Ok(())
}

fn print_help() {
    eprintln!(
        "Available commands:\n\
        - build: build STM32 application\n\
        - size: show application size\n\
        - flash: flash executable to the board"
    );
}

fn project_root() -> PathBuf {
    Path::new(&env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(1)
        .unwrap()
        .to_path_buf()
}
