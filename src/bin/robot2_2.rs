use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use drobo_interfaces::msg::PointDrive;
use p9n_interface_2024::p9n_interface_kai;

use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::sensor_msgs,
    pr_info,
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



#[async_std::main]
async fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("p9n_robot2_2", None, Default::default())?;

    let subscriber = node.create_subscriber::<sensor_msgs::msg::Joy>("joy", None)?;
    let robot2_2_publisher = node.create_publisher::<PointDrive>("/point_2_2", None)?;
    let logger = Logger::new("p9n_interface_2024");

    let pub_msg = Arc::new(Mutex::new(sensor_msgs::msg::Joy::new().unwrap()));
    let sub_msg = pub_msg.clone();

    let mut dualsense_state: [bool; 15] = [false; 15];
    let mut robot2_2_msg = PointDrive::new().unwrap();
    let mut arms_angle = [25, 25, 25, 25];
    let mut arm_mode = 1;

    let task_pub = async_std::task::spawn(async move {
        loop {
            laborer(
                &logger,
                &mut dualsense_state,
                &mut robot2_2_msg,
                &mut arms_angle,
                &mut arm_mode,
                &robot2_2_publisher,
                pub_msg.clone(),
            );
            async_std::task::sleep(Duration::from_millis(10)).await;
        }
    });
    let task_sub = async_std::task::spawn(receiver(subscriber, sub_msg));

    task_pub.await;
    task_sub.await?;

    Ok(())
}

async fn receiver(
    mut subscriber: Subscriber<sensor_msgs::msg::Joy>,
    sub_msg: Arc<Mutex<sensor_msgs::msg::Joy>>,
) -> Result<(), DynError> {
    loop {
        let _msg = subscriber.recv().await?;
        *sub_msg.lock().unwrap() = _msg.get_owned().unwrap();
    }
}

fn laborer(
    logger: &Logger,
    dualsense_state: &mut [bool],
    robot2_2_msg: &mut PointDrive,
    arms_angle: &mut [i32],
    arm_mode:&mut isize,
    robot2_2_publisher: &Publisher<PointDrive>,
    sub_msg: Arc<Mutex<sensor_msgs::msg::Joy>>,
) {
    pr_info!(logger, "Wall!");
    let binding = sub_msg.lock().unwrap();
    let mut p9n = p9n_interface_kai::PlaystationInterface::new(&binding);
    p9n.set_joy_msg(&binding);


    if *arm_mode == 0{
        match arm_mode0(&p9n, p9n.pressed_l2(), dualsense_state, DualsenseState::L2) {
            -2 => {} ,
            other =>{
                robot2_2_msg.md0=set_arm_angle(&mut arms_angle[0], other) as i16;
                pr_info!(logger, "arm0_angle: {}", robot2_2_msg.md0);
                let _  = robot2_2_publisher.send(&robot2_2_msg);
            }
        }
    
        match arm_mode0(&p9n, p9n.pressed_r1(), dualsense_state, DualsenseState::R1) {
            -2 => {} ,
            other =>{
                robot2_2_msg.md3=set_arm_angle(&mut arms_angle[3], other);
                pr_info!(logger, "arm3_angle: {}", robot2_2_msg.md3);
                let _  = robot2_2_publisher.send(&robot2_2_msg);
            }
        }
    
        match arm_mode0(&p9n, p9n.pressed_l1(), dualsense_state, DualsenseState::L1) {
            -2 => {} ,
            other =>{
                robot2_2_msg.md1=set_arm_angle(&mut arms_angle[1], -other) as i16;
                pr_info!(logger, "arm1_angle: {}", robot2_2_msg.md1);
                let _  = robot2_2_publisher.send(&robot2_2_msg);
            }
        }
    
        match arm_mode0(&p9n, p9n.pressed_r2(), dualsense_state, DualsenseState::R2) {
            -2 => {} ,
            other =>{
                robot2_2_msg.md2=set_arm_angle(&mut arms_angle[2], -other);
                pr_info!(logger, "arm2_angle: {}", robot2_2_msg.md2);
                let _  = robot2_2_publisher.send(&robot2_2_msg);
            }
        }
    }

    if *arm_mode == 1{
        match arm_mode1(p9n.pressed_l2(), dualsense_state, DualsenseState::L2) {
            -2 => {} ,
            other =>{
                robot2_2_msg.md0=other as i16;
                arms_angle[0]= other as i32;
                let _  = robot2_2_publisher.send(&robot2_2_msg);
            }
        }
    
        match arm_mode1(p9n.pressed_r1(), dualsense_state, DualsenseState::R1) {
            -2 => {} ,
            other =>{
                robot2_2_msg.md3=other as i8;
                arms_angle[3]= other as i32;
                let _  = robot2_2_publisher.send(&robot2_2_msg);
            }
        }
    
        match arm_mode1(p9n.pressed_l1(), dualsense_state, DualsenseState::L1) {
            -2 => {} ,
            other =>{
                robot2_2_msg.md1=(150 - other) as i16;
                arms_angle[1]= (150 - other) as i32;
                let _  = robot2_2_publisher.send(&robot2_2_msg);
            }
        }
    
        match arm_mode1(p9n.pressed_r2(), dualsense_state, DualsenseState::R2) {
            -2 => {} ,
            other =>{
                robot2_2_msg.md2=(150 - other) as i8;
                arms_angle[2]= (150 - other) as i32;
                let _  = robot2_2_publisher.send(&robot2_2_msg);
            }
        }

    }

    if p9n.pressed_circle(){
        dualsense_state[DualsenseState::CIRCLE] = true;
    }
    
    if p9n.pressed_circle(){
        *arm_mode = 0;
    }
    if p9n.pressed_square(){
        *arm_mode = 1;
    }

    if p9n.pressed_triangle() && !dualsense_state[DualsenseState::TRIANGLE] {
        pr_info!(logger, "triangle");
        dualsense_state[DualsenseState::TRIANGLE] = true;
        robot2_2_msg.md5 = if !p9n.pressed_cross() { 127 } else { -128 };
        let _ = robot2_2_publisher.send(&robot2_2_msg);
    }
    if !p9n.pressed_triangle() && dualsense_state[DualsenseState::TRIANGLE] {
        pr_info!(logger, "reverse triangle");
        dualsense_state[DualsenseState::TRIANGLE] = false;
    }
}

fn set_arm_angle(arm_angle: &mut i32, target: i32) -> i8{
    if (*arm_angle + target) > 125 {
        *arm_angle = 125;
    } else if (*arm_angle + target) < 25 {
        *arm_angle = 25;
    } else {
        *arm_angle += target;
    }
    return *arm_angle as i8;
}

fn arm_mode0(p9n:&p9n_interface_kai::PlaystationInterface,p9n_pressed:bool,dualsense_state:&mut[bool],button:usize,)->i32{
    if p9n_pressed {
        dualsense_state[button] = true;
            return  if !p9n.pressed_cross() { 1 } else { -1 };
    }
    else if !p9n_pressed && dualsense_state[button] {
        dualsense_state[button] = false;
        return 0;
    }

    -2
}

fn arm_mode1(p9n_pressed:bool,dualsense_state:&mut[bool],button:usize)->isize{

    let   arm_angle = 
    if p9n_pressed {
        dualsense_state[button] = true;
        125 
    }
    else if !p9n_pressed && dualsense_state[button] {
        dualsense_state[button] = false;
        75
    }else{
        return -2;
    };
    
    arm_angle



}