use std::collections::HashMap;
use std::env;
use std::fs::{self, File};
use std::io::{prelude::*, BufReader};
use std::path::PathBuf;

const COLORS: &[&str] = &[
    "#000000",
    "#FF0000",
    "#00FF00",
    "#FFFF00",
    "#0000FF",
    "#FF00FF",
    "#00FFFF",
    "#FFFFFF",
];

fn main() {
    const FILENAME: &'static str = "superpollo.xpm";

    println!("cargo:rerun-if-changed={}", FILENAME);

    // wow what a shit parser

    let mut lines = BufReader::new(File::open(FILENAME).unwrap()).lines().map(Result::unwrap);

    assert!(lines.next().unwrap().contains("XPM"));
    assert!(lines.next().unwrap().contains("static char"));

    let mut lines = lines.map(|l| l.trim_matches(&['"', ','][..]).to_owned());

    let args: Vec<usize> = lines.next().unwrap()
        .split_whitespace()
        .map(|i| i.parse().unwrap())
        .collect::<Vec<_>>();

    let width = args[0];
    let height = args[1];
    let colors = args[2];

    let mut chars: HashMap<char, u8> = HashMap::new();

    for _ in 0..colors {
        let l = lines.next().unwrap();
        let ch = l.chars().next().unwrap();
        let co = l.split_whitespace().last().unwrap();

        chars.insert(
            ch,
            COLORS.iter().position(|&c| c == co).unwrap() as u8);
    }

    println!("{:?}", chars);

    let mut data: Vec<u8> = Vec::new();
    for y in 0..height {
        let l = lines.next().unwrap();
        let mut c = l.chars();

        println!("{}", l.len());

        for x in 0..width / 2 {
            let c1 = c.next().unwrap();
            let c2 = c.next().unwrap();

            let c1 = *chars.get(&c1).unwrap();
            let c2 = *chars.get(&c2).unwrap();

            data.push(c1 | (c2 << 4));
        }
    }

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::write(out_path.join("image.bin"), data).unwrap();
}