use p9n_interface_2024::p9n_interface_kai;
use std::{thread, time};
use drobo_interfaces::msg::{MdLibMsg, SdLibMsg, SmdLibMsg};
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::{sensor_msgs, geometry_msgs},
    pr_info,
    selector::Selector,
    topic::{publisher::Publisher, subscriber::Subscriber},
};

#[allow(non_snake_case)]
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
    let node = ctx.create_node("p9n_robot1", None, Default::default())?;

    let selector = ctx.create_selector()?;
    let subscriber = node.create_subscriber::<sensor_msgs::msg::Joy>("joy", None)?;

    let md_publisher = node.create_publisher::<MdLibMsg>("/md_driver_topic", None)?;
    let sd_publisher = node.create_publisher::<SdLibMsg>("/sd_driver_topic", None)?;
    let smd_publisher = node.create_publisher::<SmdLibMsg>("/smd_driver_topic", None)?;
    let pose_publisher = node.create_publisher::<geometry_msgs::msg::Vector3>("goal_pose", None)?;

    worker(selector, subscriber, md_publisher, sd_publisher, smd_publisher, pose_publisher)?;
    Ok(())
}

fn worker(
    mut selector: Selector,
    subscriber: Subscriber<sensor_msgs::msg::Joy>,
    md_publisher: Publisher<MdLibMsg>,
    sd_publisher: Publisher<SdLibMsg>,
    smd_publisher: Publisher<SmdLibMsg>,
    pose_publisher: Publisher<geometry_msgs::msg::Vector3>
) -> Result<(), DynError> {
    let logger = Logger::new("p9n_interface_2024");
    let mut dualsense_state: [bool; 15] = [false; 15];

    let mut md_msg = MdLibMsg::new().unwrap();
    let mut sd_msg = SdLibMsg::new().unwrap();
    let mut smd_msg = SmdLibMsg::new().unwrap();
    let origin_left_pose_msg = geometry_msgs::msg::Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let origin_right_pose_msg = geometry_msgs::msg::Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let robot2_1_pose_msg = geometry_msgs::msg::Vector3 {
        x: 1.0,
        y: -1.0,
        z: 0.0,
    };
    let robot2_3_pose_msg = geometry_msgs::msg::Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    let mut exhaust_solenoid_power = 0;

    selector.add_subscriber(
        subscriber,
        Box::new(move |_msg| {
            let mut p9n = p9n_interface_kai::PlaystationInterface::new(&_msg);
            p9n.set_joy_msg(&_msg);

            if p9n.pressed_start() && !dualsense_state[DualsenseState::START]{
                pr_info!(logger, "start");
                dualsense_state[DualsenseState::START] = true;
                let _ = pose_publisher.send(&robot2_1_pose_msg);
            }
            if !p9n.pressed_start() && dualsense_state[DualsenseState::START]{
                pr_info!(logger, "reverse start");
                dualsense_state[DualsenseState::START] = false;
            }
            if p9n.pressed_cross() && !dualsense_state[DualsenseState::CROSS]{
                pr_info!(logger, "cross");
                dualsense_state[DualsenseState::CROSS] = true;
                let _ = pose_publisher.send(&origin_right_pose_msg);
            }
            if !p9n.pressed_cross() && dualsense_state[DualsenseState::CROSS]{
                pr_info!(logger, "reverse cross");
                dualsense_state[DualsenseState::CROSS] = false;
            }
            if p9n.pressed_circle() && !dualsense_state[DualsenseState::CIRCLE]{
                pr_info!(logger, "circle");
                dualsense_state[DualsenseState::CIRCLE] = true;
                let _ = pose_publisher.send(&robot2_1_pose_msg);
            }
            if !p9n.pressed_circle() && dualsense_state[DualsenseState::CIRCLE]{
                pr_info!(logger, "reverse circle");
                dualsense_state[DualsenseState::CIRCLE] = false;
            }
            if p9n.pressed_square() && !dualsense_state[DualsenseState::SQUARE]{
                pr_info!(logger, "square");
                dualsense_state[DualsenseState::SQUARE] = true;
                let _ = pose_publisher.send(&origin_left_pose_msg);
            }
            if !p9n.pressed_square() && dualsense_state[DualsenseState::SQUARE]{
                pr_info!(logger, "reverse square");
                dualsense_state[DualsenseState::SQUARE] = false;
            }

            if p9n.pressed_dpad_right() && !dualsense_state[DualsenseState::D_PAD_RIGHT]{
                pr_info!(logger, "right");
                dualsense_state[DualsenseState::D_PAD_RIGHT] = true;
                sd_msg.address = 0x00;
                sd_msg.port = 1;
                sd_msg.power1 = 1000;
                let _ = sd_publisher.send(&sd_msg);
            }
            if !p9n.pressed_dpad_right() && dualsense_state[DualsenseState::D_PAD_RIGHT] {
                pr_info!(logger, "reverse right");
                dualsense_state[DualsenseState::D_PAD_RIGHT] = false;
                sd_msg.address = 0x00;
                sd_msg.port = 1;
                sd_msg.power1 = 0;
                let _ = sd_publisher.send(&sd_msg);
                sd_msg.port = 0;
                exhaust_solenoid_power = 0;
                sd_msg.power1 = exhaust_solenoid_power;
                let _ = sd_publisher.send(&sd_msg);
            }


            // 右側
            // 　そのまま
            if p9n.pressed_dpad_up(){
                pr_info!(logger, "up");
                dualsense_state[DualsenseState::D_PAD_UP] = true;
                md_msg.address = 0x05;
                md_msg.mode = 5;
                md_msg.phase = false;
                md_msg.power = 1000;
                let _ = md_publisher.send(&md_msg);
            }
            // if !p9n.pressed_dpad_up() && dualsense_state[DualsenseState::D_PAD_UP] {
            //     pr_info!(logger, "reverse up");
            //     dualsense_state[DualsenseState::D_PAD_UP] = false;
            //     md_msg.address = 0x05;
            //     md_msg.mode = 2;
            //     md_msg.phase = false;
            //     md_msg.power = 0;
            //     let _ = md_publisher.send(&md_msg);
            // }

            // そのまま
            if p9n.pressed_dpad_down() &&!dualsense_state[DualsenseState::D_PAD_DOWN]{
                pr_info!(logger, "down");
                dualsense_state[DualsenseState::D_PAD_DOWN] = true;
                sd_msg.address = 0x05;
                sd_msg.port = 0;
                sd_msg.power1 = 1000;
                let _ = sd_publisher.send(&sd_msg);
            }
            if !p9n.pressed_dpad_down() && dualsense_state[DualsenseState::D_PAD_DOWN] {
                pr_info!(logger, "reverse down");
                dualsense_state[DualsenseState::D_PAD_DOWN] = false;
                sd_msg.address = 0x05;
                sd_msg.port = 0;
                sd_msg.power1 = 0;
                let _ = sd_publisher.send(&sd_msg);
            }

            // 左射出
            if p9n.pressed_r1()&& p9n.pressed_triangle()&& !dualsense_state[DualsenseState::R1]{
                pr_info!(logger, "r2_down");
                dualsense_state[DualsenseState::R1] = true;

                // サーボ開ける
                smd_msg.address = 6;
                smd_msg.port = 1;
                smd_msg.mode=1;
                smd_msg.angle=180;
                let _ = smd_publisher.send(&smd_msg);

                let millis = time::Duration::from_millis(1000);
                thread::sleep(millis);

                // mainバルブ
                sd_msg.address = 0;
                sd_msg.port = 1;
                sd_msg.power1 = 1000;
                let _ = sd_publisher.send(&sd_msg);

                let millis = time::Duration::from_millis(500);
                thread::sleep(millis);

                sd_msg.address = 0;
                sd_msg.port = 1;
                sd_msg.power1 = 0;
                let _ = sd_publisher.send(&sd_msg);

                smd_msg.address = 6;
                smd_msg.port = 1;
                smd_msg.mode=1;
                smd_msg.angle=90;
                let _ = smd_publisher.send(&smd_msg);

            }
            if !p9n.pressed_r1() && dualsense_state[DualsenseState::R1] {
                dualsense_state[DualsenseState::R1] = false;
            }
            // 左装填
            if p9n.pressed_l1() && p9n.pressed_triangle() && !dualsense_state[DualsenseState::L1]{
                pr_info!(logger, "down");
                dualsense_state[DualsenseState::L1] = true;

                smd_msg.address = 6;
                smd_msg.port = 1;
                smd_msg.mode=1;
                smd_msg.angle=90;
                let _ = smd_publisher.send(&smd_msg);

                sd_msg.address = 1;
                sd_msg.power1 = 1000;
                sd_msg.address = 1;
                sd_msg.power2 = 1000;
                let _ = sd_publisher.send(&sd_msg);

                let millis = time::Duration::from_millis(3000);
                thread::sleep(millis);

                sd_msg.address = 1;
                sd_msg.power1 = 0;
                sd_msg.address = 1;
                sd_msg.power2 = 0;
                let _ = sd_publisher.send(&sd_msg);

            }
            if !p9n.pressed_l1() && dualsense_state[DualsenseState::L1] {
                // pr_info!(logger, "reverse down");
                dualsense_state[DualsenseState::L1] = false;
            }

            // 左の射出2
            if p9n.pressed_r2()&& p9n.pressed_triangle()&& !dualsense_state[DualsenseState::R2]{
                pr_info!(logger, "r2_down");
                dualsense_state[DualsenseState::R2] = true;

                // サーボ開ける
                smd_msg.address = 6;
                smd_msg.port = 1;
                smd_msg.mode=1;
                smd_msg.angle=180;
                let _ = smd_publisher.send(&smd_msg);

                let millis = time::Duration::from_millis(400);
                thread::sleep(millis);

                smd_msg.address = 6;
                smd_msg.port = 1;
                smd_msg.mode=1;
                smd_msg.angle=90;
                let _ = smd_publisher.send(&smd_msg);

                let millis = time::Duration::from_millis(600);
                thread::sleep(millis);
                // mainバルブ
                sd_msg.address = 0;
                sd_msg.port = 1;
                sd_msg.power1 = 1000;
                let _ = sd_publisher.send(&sd_msg);

                let millis = time::Duration::from_millis(500);
                thread::sleep(millis);

                sd_msg.address = 0;
                sd_msg.port = 1;
                sd_msg.power1 = 0;
                let _ = sd_publisher.send(&sd_msg);

            }
            if !p9n.pressed_r2() && dualsense_state[DualsenseState::R2] {
                dualsense_state[DualsenseState::R2] = false;
            }

        }),
    );
    loop {
        selector.wait()?;
    }
}
