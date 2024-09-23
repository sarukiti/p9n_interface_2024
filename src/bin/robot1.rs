use p9n_interface_2024::p9n_interface;

use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::sensor_msgs,
    pr_info,
    selector::Selector,
    topic::{publisher::Publisher, subscriber::Subscriber},
};
use drobo_interfaces::msg::{MdLibMsg, SdLibMsg};

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

    worker(
        selector,
        subscriber,
        md_publisher,
        sd_publisher, 
    )?;
    Ok(())
}

fn worker(
    mut selector: Selector,
    subscriber: Subscriber<sensor_msgs::msg::Joy>,
    md_publisher: Publisher<MdLibMsg>,
    sd_publisher: Publisher<SdLibMsg>,
) -> Result<(), DynError> {
    let mut p9n = p9n_interface::PlaystationInterface::new(sensor_msgs::msg::Joy::new().unwrap());
    let logger = Logger::new("p9n_interface_2024");
    let mut dualsense_state: [bool; 15] = [false; 15];
    
    let mut md_msg = MdLibMsg::new().unwrap();
    let mut sd_msg = SdLibMsg::new().unwrap();

    selector.add_subscriber(
        subscriber,
        Box::new(move |_msg| {
            p9n.set_joy_msg(_msg.get_owned().unwrap());

            if p9n.pressed_dpad_left(){
                pr_info!(logger, "left");
                dualsense_state[DualsenseState::D_PAD_LEFT] = true;
                sd_msg.address = 0x00;
                sd_msg.port = 0;
                sd_msg.power1 = 1000;
                let _ = sd_publisher.send(&sd_msg);
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
                sd_msg.power1 = 0;
                let _ = sd_publisher.send(&sd_msg);

            }
            if p9n.pressed_dpad_up() && !dualsense_state[DualsenseState::D_PAD_UP]{
                pr_info!(logger, "up");
                dualsense_state[DualsenseState::D_PAD_UP] = true;
                md_msg.address = 0x05;
                md_msg.mode = 2;
                md_msg.phase = p9n.pressed_cross();
                md_msg.power = 1000;
                let _ = md_publisher.send(&md_msg);
            }
            if !p9n.pressed_dpad_up() && dualsense_state[DualsenseState::D_PAD_UP] {
                pr_info!(logger, "reverse up");
                dualsense_state[DualsenseState::D_PAD_UP] = false;
                md_msg.address = 0x05;
                md_msg.mode = 2;
                md_msg.phase = false;
                md_msg.power = 0;
                let _ = md_publisher.send(&md_msg);
            }
            if p9n.pressed_dpad_down() && !dualsense_state[DualsenseState::D_PAD_DOWN]{
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
        }),
    );
    loop {
        selector.wait()?;
    }
}
