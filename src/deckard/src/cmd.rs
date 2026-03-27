use crate::core::{Rover, WheelOrientation, NodeCommand, Operation};
use std::f32::consts::PI;

#[derive(Debug)]
pub enum CmdErr {
    Timeout,
    SyntaxError,
    ValueError,
    HardwareError,
    NotImplemented
}

// Each struct that implements this corresponds to a class of related commands that have the same
// brief and help strings
pub trait Cmd {
    fn get_keywords(&self) -> Vec<&'static str>;
    fn get_brief(&self) -> &'static str;
    fn get_help(&self) -> &'static str;
    fn run(&self, args: Vec<&str>, rover: &mut Rover) -> Result<(), CmdErr>;
}

pub struct Quit {}
impl Cmd for Quit {
    fn get_keywords(&self) -> Vec<&'static str> {
        Vec::from(["quit", "!", "exit"])
    }

    fn get_brief(&self) -> &'static str {
        "Exits the REPL gracefully."
    }

    fn get_help(&self) -> &'static str {
        "Usage: quit\n\
        Aliases: !, exit\n\
        Exits the REPL gracefully."
    }

    fn run(&self, _args: Vec<&str>, _rover: &mut Rover) -> Result<(), CmdErr> {
        panic!("quit command should never be run in the normal way");
    }
}

pub struct Help {}
impl Cmd for Help {
    fn get_keywords(&self) -> Vec<&'static str> {
        Vec::from(["help", "h", "?"])
    }

    fn get_brief(&self) -> &'static str {
        "Provides help on how to use this program."
    }

    fn get_help(&self) -> &'static str {
        "Usage: help
                help <command>\n\
        Aliases: h, ?\n\
        Just try it and see what happens."
    }

    fn run(&self, _args: Vec<&str>, _rover: &mut Rover) -> Result<(), CmdErr> {
        panic!("help command should never be run in the normal way");
    }
}

pub struct Stop {}
impl Cmd for Stop {
    fn get_keywords(&self) -> Vec<&'static str> {
        Vec::from(["stop", "halt", "x", "X"])
    }

    fn get_brief(&self) -> &'static str {
        "Immediately halts the rover drivebase."
    }

    fn get_help(&self) -> &'static str {
        "Usage: stop\n\
        Aliases: halt, x, X\n\
        Immediately halts the rover drivebase. All 'move' commands should be eventually followed by this command."
    }

    fn run(&self, _args: Vec<&str>, rover: &mut Rover) -> Result<(), CmdErr> {
        rover.cmd_tx.send(NodeCommand {
            op: Operation::DriveVesc,
            values: [0.0; 4] // drive: position 0
        }).unwrap();
        println!("STOPPED!");
        rover.is_stopped = true;
        Ok(())
    }
}

const LOW: f32 = 0.03;
const NORMAL: f32 = 0.06;
const HIGH: f32 = 0.09;

fn parse_speed(mut s: &str) -> Option<f32> {
    let mut f = 1.0;
    if s.len() < 1 {
        return None;
    }
    if s.chars().nth(0).unwrap() == '-' {
        f = -1.0;
        s = &s[1..];
    }
    match s {
        "hi" => Some(f * HIGH),
        "norm" => Some(f * NORMAL),
        "slo" => Some(f * LOW),
        _ => None
    }
}

pub struct Move {}
impl Cmd for Move {
    fn get_keywords(&self) -> Vec<&'static str> {
        Vec::from(["move", "m", "mv", "w", "a", "s", "d", "q", "e", "W", "A", "S", "D", "Q", "E"])
    }

    fn get_brief(&self) -> &'static str {
        "Moves the rover in a variety of interesting ways."
    }

    fn get_help(&self) -> &'static str {
        "Usage: move straight <speed>\n\
                move strafe <angle> <speed>\n\
                move circle <speed>\n\
        MAKE SURE YOU KNOW HOW TO USE THE STOP COMMAND BEFORE YOU RUN THIS.\n\
        Aliases for specific move command variants: w, a, s, d, q, e (and CAPS variants)\n\
        'straight' aliased by 'st', 'strafe' aliased by 'sf', 'circle' aliased by 'cl'\n\
        <speed>: either 'lo' (0.03), 'norm' (0.06) or 'hi' (0.09); prefix with '-' to go backwards\n\
        <angle>: integer, in degrees; range is (-90, 90]\n\
        Default for circle is clockwise; use 'backwards' speed to go anticlockwise"
    }

    fn run(&self, args: Vec<&str>, rover: &mut Rover) -> Result<(), CmdErr> {
        let kw = args[0];
        let kw_lower = kw.to_lowercase();
        let (target_orientation, speed): (WheelOrientation, f32) = match kw_lower.as_str() {
            "move" | "mv" | "m" => {
                if args.len() < 3 {
                    eprintln!("Not enough arguments for 'move', need at least 3");
                    return Err(CmdErr::SyntaxError)
                }
                match args[1] {
                    "st" | "straight" => {
                        match parse_speed(args[2]) {
                            Some(speed) => (WheelOrientation::Aligned(0.0), speed),
                            None => {
                                eprintln!("Unrecognised speed '{}'", args[2]);
                                return Err(CmdErr::SyntaxError);
                            }
                        }
                    }
                    "sf" | "strafe" => {
                        let target_angle: f32 = match args[2].parse::<isize>() {
                            Ok(target_angle_int) => {
                                if target_angle_int > 90 || target_angle_int <= -90 {
                                    eprintln!("The angle {} is out of range (-90, 90]", target_angle_int);
                                    return Err(CmdErr::ValueError);
                                }
                                target_angle_int as f32 * PI / 180.0
                            }
                            Err(_) => { eprintln!("{} is not a valid angle", args[2]); return Err(CmdErr::SyntaxError) }
                        };
                        if args.len() < 4 { eprintln!("Not enough arguments for 'move strafe', need 4"); return Err(CmdErr::SyntaxError) };
                        match parse_speed(args[3]) {
                            Some(speed) => (WheelOrientation::Aligned(target_angle), speed),
                            None => {
                                eprintln!("Unrecognised speed '{}'", args[3]);
                                return Err(CmdErr::SyntaxError);
                            }
                        }
                    }
                    "cl" | "circle" => {
                        match parse_speed(args[3]) {
                            Some(speed) => (WheelOrientation::RotateInPlace, speed),
                            None => {
                                eprintln!("Unrecognised speed '{}'", args[3]);
                                return Err(CmdErr::SyntaxError);
                            }
                        }
                    }
                    _ => {
                        eprintln!("Syntax error on 'move': unrecognised move mode {}", args[1]);
                        return Err(CmdErr::SyntaxError)
                    }
                }
            }
            "w" | "s" => {
                if args[0].chars().nth(0).unwrap().is_lowercase() {
                    (WheelOrientation::Aligned(0.0), LOW)
                } else {
                    (WheelOrientation::Aligned(0.0), NORMAL)
                }
            }
            "q" | "e" => {
                if args[0].chars().nth(0).unwrap().is_lowercase() {
                    (WheelOrientation::Aligned(90.0 * PI / 180.0), LOW)
                } else {
                    (WheelOrientation::Aligned(90.0 * PI / 180.0), NORMAL)
                }
            }
            "a" | "d" => {
                if args[0].chars().nth(0).unwrap().is_lowercase() {
                    (WheelOrientation::RotateInPlace, LOW)
                } else {
                    (WheelOrientation::RotateInPlace, NORMAL)
                }
            }
            _ => {
                return Err(CmdErr::SyntaxError);
            }
        };
        // handle pre-drive steering
        println!("Steering into position...");
        match target_orientation {
            WheelOrientation::Aligned(target_angle) => {
                rover.cmd_tx.send(NodeCommand {
                    op: Operation::DriveStepper,
                    values: [target_angle; 4]
                }).unwrap();
            }
            WheelOrientation::RotateInPlace => {
                const ROTATE_ANGLES: [f32; 4] = [-1.04, 1.06, 1.06, -1.04];
                rover.cmd_tx.send(NodeCommand {
                    op: Operation::DriveStepper,
                    values: ROTATE_ANGLES
                }).unwrap();
            }
        }
        // block on command completing
        rover.cmd_tx.send(NodeCommand {
            op: Operation::NotifyWhenDone,
            values: [f32::NAN; 4]
        }).unwrap();
        // TODO: sus
        rover.notif_rx.recv().unwrap().unwrap();
        println!("Wheels are now in position. Driving!");
        rover.wheels = target_orientation;
        // then move
        rover.cmd_tx.send(NodeCommand {
            op: Operation::DriveVesc,
            values: [speed; 4]
        }).unwrap();
        rover.is_stopped = false;
        Ok(())
    }
}
