mod p9n_interface;
mod ps5_dualsense;

use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::sensor_msgs,
    pr_fatal, pr_info,
    selector::Selector,
    topic::{publisher::Publisher, subscriber::Subscriber},
};
use std::{rc::Rc, time::Duration};
use drobo_interfaces::msg::{MdLibMsg, SdLibMsg, PointDrive};

pub mod DualsenseState {
    pub const SQUARE: usize = 0;
    pub const CIRCLE: usize = 1;
    pub const TRIANGLE: usize = 2;
    pub const CROSS: usize = 3;
    pub const L1: usize = 4;
    pub const L2: usize = 5;
    pub const R1: usize = 6;
    pub const R2: usize = 7;
    pub const D_PAD_UP: usize = 8;
    pub const D_PAD_DOWN: usize = 9;
    pub const D_PAD_LEFT: usize = 10;
    pub const D_PAD_RIGHT: usize = 11;
    pub const START: usize = 12;
    pub const SELECT: usize = 13;
    pub const PS: usize = 14;
}

fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("p9n_interface_2024", None, Default::default())?;

    let selector = ctx.create_selector()?;
    let subscriber = node.create_subscriber::<sensor_msgs::msg::Joy>("joy", None)?;

    let robot2_1_publisher = node.create_publisher::<PointDrive>("/point_2_1", None)?;
    let robot2_2_publisher = node.create_publisher::<PointDrive>("/point_2_2", None)?;
    let robot2_3_publisher = node.create_publisher::<PointDrive>("/point_2_3", None)?;
    let robot2_4_publisher = node.create_publisher::<PointDrive>("/point_2_4", None)?;

    let md_publisher = node.create_publisher::<MdLibMsg>("/md_driver_topic", None)?;
    let sd_publisher = node.create_publisher::<SdLibMsg>("/sd_driver_topic", None)?;

    worker(
        selector,
        subscriber,
        md_publisher,
        sd_publisher, 
        robot2_1_publisher,
        robot2_2_publisher,
        robot2_3_publisher,
        robot2_4_publisher,
    )?;
    Ok(())
}

fn worker(
    mut selector: Selector,
    subscriber: Subscriber<sensor_msgs::msg::Joy>,
    md_publisher: Publisher<MdLibMsg>,
    sd_publisher: Publisher<SdLibMsg>,
    robot2_1_publisher: Publisher<PointDrive>,
    robot2_2_publisher: Publisher<PointDrive>,
    robot2_3_publisher: Publisher<PointDrive>,
    robot2_4_publisher: Publisher<PointDrive>,
) -> Result<(), DynError> {
    let mut p9n = p9n_interface::PlaystationInterface::new(sensor_msgs::msg::Joy::new().unwrap());
    let logger = Rc::new(Logger::new("p9n_interface_2024"));
    let logger2 = logger.clone();
    let mut dualsense_state: [bool; 15] = [false; 15];
    let mut robot_num: u8 = 0;

    let mut md_msg = MdLibMsg::new().unwrap();
    let mut sd_msg = SdLibMsg::new().unwrap();
    let mut robot2_1_msg = PointDrive::new().unwrap();
    let mut robot2_2_msg = PointDrive::new().unwrap();
    let mut robot2_3_msg = PointDrive::new().unwrap();
    let mut robot2_4_msg = PointDrive::new().unwrap();

    selector.add_subscriber(
        subscriber,
        Box::new(move |_msg| {
            p9n.set_joy_msg(_msg.get_owned().unwrap());

            if p9n.pressed_select() && !dualsense_state[DualsenseState::SELECT] {
                dualsense_state[DualsenseState::SELECT] = true;
                robot_num = (robot_num + 1) % 5;
                pr_info!(logger, "now, controlling robot No.{}", robot_num);
            } 
            if !p9n.pressed_select() && dualsense_state[DualsenseState::SELECT] {
                dualsense_state[DualsenseState::SELECT] = false;
            }
            if p9n.pressed_dpad_left(){
                pr_info!(logger, "left");
                dualsense_state[DualsenseState::D_PAD_LEFT] = true;
                if robot_num == 0 {
                    sd_msg.address = 0x00;
                    sd_msg.port = 0;
                    sd_msg.power1 = 1000;
                    let _ = sd_publisher.send(&sd_msg);
                } else if robot_num == 2 {
                    robot2_2_msg.md0 = if !p9n.pressed_cross() {1000} else {-1000};
                    let _ = robot2_2_publisher.send(&robot2_2_msg);
                }
            } 
            if !p9n.pressed_dpad_left() && dualsense_state[DualsenseState::D_PAD_LEFT] {
                pr_info!(logger, "reverse left");
                dualsense_state[DualsenseState::D_PAD_LEFT] = false;
                if robot_num == 0 {
                    sd_msg.address = 0x00;
                    sd_msg.port = 0;
                    sd_msg.power1 = 0;
                    let _ = sd_publisher.send(&sd_msg);
                } else if robot_num == 2 {
                    robot2_2_msg.md0 = 0;
                    let _ = robot2_2_publisher.send(&robot2_2_msg);
                }
            }
            if p9n.pressed_dpad_right() && !dualsense_state[DualsenseState::D_PAD_RIGHT]{
                pr_info!(logger, "right");
                dualsense_state[DualsenseState::D_PAD_RIGHT] = true;
                if robot_num == 0 {
                    sd_msg.address = 0x00;
                    sd_msg.port = 1;
                    sd_msg.power1 = 1000;
                    let _ = sd_publisher.send(&sd_msg);
                } else if robot_num == 2 {
                    robot2_2_msg.md1 = if !p9n.pressed_cross() {1000} else {-1000};
                    let _ = robot2_2_publisher.send(&robot2_2_msg);
                }
            } 
            if !p9n.pressed_dpad_right() && dualsense_state[DualsenseState::D_PAD_RIGHT] {
                pr_info!(logger, "reverse right");
                dualsense_state[DualsenseState::D_PAD_RIGHT] = false;
                if robot_num == 0 {
                    sd_msg.address = 0x00;
                    sd_msg.port = 1;
                    sd_msg.power1 = 0;
                    let _ = sd_publisher.send(&sd_msg);
                } else if robot_num == 2 {
                    robot2_2_msg.md1 = 0;
                    let _ = robot2_2_publisher.send(&robot2_2_msg);
                }
            }
            if p9n.pressed_square() && !dualsense_state[DualsenseState::SQUARE] {
                pr_info!(logger, "square");
                dualsense_state[DualsenseState::SQUARE] = true;
                if robot_num == 2 {
                    robot2_2_msg.md3 = if robot2_2_msg.md3 <= 0 {127} else {-128};
                    let _ = robot2_2_publisher.send(&robot2_2_msg);
                }

            }
            if !p9n.pressed_square() && dualsense_state[DualsenseState::SQUARE] {
                pr_info!(logger, "reverse square");
                dualsense_state[DualsenseState::SQUARE] = false;
            }
            if p9n.pressed_triangle() && !dualsense_state[DualsenseState::TRIANGLE] {
                pr_info!(logger, "triangle");
                dualsense_state[DualsenseState::TRIANGLE] = true;
                if robot_num == 1 {
                    robot2_1_msg.md5 = if robot2_1_msg.md5 <= 0 {127} else {-128};
                    let _ = robot2_1_publisher.send(&robot2_1_msg);
                } else if robot_num == 2 {
                    robot2_2_msg.md5 = if robot2_2_msg.md5 <= 0 {127} else {-128};
                    let _ = robot2_2_publisher.send(&robot2_2_msg);
                }
            }
            if !p9n.pressed_triangle() && dualsense_state[DualsenseState::TRIANGLE] {
                pr_info!(logger, "reverse triangle");
                dualsense_state[DualsenseState::TRIANGLE] = false;
            }
            if p9n.pressed_circle() && !dualsense_state[DualsenseState::CIRCLE] {
                pr_info!(logger, "circle");
                dualsense_state[DualsenseState::CIRCLE] = true;
                if robot_num == 2 {
                    robot2_2_msg.md4 = if robot2_2_msg.md4 <= 0 {127} else {-128};
                    let _ = robot2_2_publisher.send(&robot2_2_msg);
                }
            }
            if !p9n.pressed_circle() && dualsense_state[DualsenseState::CIRCLE] {
                pr_info!(logger, "reverse circle");
                dualsense_state[DualsenseState::CIRCLE] = false;
            }
            if p9n.pressed_dpad_up() {
                pr_info!(logger, "up");
                if robot_num == 0 {
                    if !dualsense_state[DualsenseState::D_PAD_UP] {
                        md_msg.address = 0x05;
                        md_msg.mode = 2;
                        md_msg.phase = p9n.pressed_cross();
                        md_msg.power = 1000;
                        let _ = md_publisher.send(&md_msg);
                    }
                } else if robot_num == 1 {
                    robot2_1_msg.md5 = if !p9n.pressed_cross() {127} else {-128};
                    let _ = robot2_1_publisher.send(&robot2_1_msg);
                } else if robot_num == 2 {
                    robot2_2_msg.md5 = if !p9n.pressed_cross() {127} else {-128};
                    let _ = robot2_2_publisher.send(&robot2_2_msg);
                } else if robot_num == 3 {
                    robot2_3_msg.md5 = if !p9n.pressed_cross() {127} else {-128};
                    let _ = robot2_3_publisher.send(&robot2_3_msg);
                }
                dualsense_state[DualsenseState::D_PAD_UP] = true;
            }
            if !p9n.pressed_dpad_up() && dualsense_state[DualsenseState::D_PAD_UP] {
                pr_info!(logger, "reverse up");
                dualsense_state[DualsenseState::D_PAD_UP] = false;
                if robot_num == 0 {
                    md_msg.address = 0x05;
                    md_msg.mode = 2;
                    md_msg.phase = false;
                    md_msg.power = 0;
                    let _ = md_publisher.send(&md_msg);
                    sd_msg.address = 0x05;
                    sd_msg.port = 0;
                    sd_msg.power1 = 1000;
                    let _ = sd_publisher.send(&sd_msg);
                } else if robot_num == 1 {
                    robot2_1_msg.md2 = 0;
                    let _ = robot2_1_publisher.send(&robot2_1_msg);
                } else if robot_num == 2 {
                    robot2_2_msg.md2 = 0;
                    let _ = robot2_2_publisher.send(&robot2_2_msg);
                } else if robot_num == 3 {
                    robot2_3_msg.md2 = 0;
                    robot2_3_msg.md3 = 0;
                    let _ = robot2_3_publisher.send(&robot2_3_msg);
                }
            }
            if p9n.pressed_l2() {
                pr_info!(logger, "l2");
                dualsense_state[DualsenseState::L2] = true;
                robot2_3_msg.md4 = if !p9n.pressed_cross() {127} else {-128};
                let _ = robot2_3_publisher.send(&robot2_3_msg);
            }
            if !p9n.pressed_l2() && dualsense_state[DualsenseState::L2] {
                pr_info!(logger, "reverse l2");
                dualsense_state[DualsenseState::L2] = false;
                robot2_3_msg.md4 = 0;
                let _ = robot2_3_publisher.send(&robot2_3_msg);
            }
            if p9n.pressed_r2() {
                pr_info!(logger, "r2");
                dualsense_state[DualsenseState::R2] = true;
                robot2_3_msg.md5 = if !p9n.pressed_cross() {127} else {-128};
                let _ = robot2_3_publisher.send(&robot2_3_msg);
            }
            if !p9n.pressed_r2() && dualsense_state[DualsenseState::R2] {
                pr_info!(logger, "reverse r2");
                dualsense_state[DualsenseState::R2] = false;
                robot2_3_msg.md5 = 0;
                let _ = robot2_3_publisher.send(&robot2_3_msg);
            }            
        }),
    );
    loop {
        selector.wait()?;
    }
}
