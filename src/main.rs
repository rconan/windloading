use dos_actors::{
    clients::{
        arrow_client::Arrow,
        fem::M1SegmentsAxialD,
        mount::{Mount, MountEncoders, MountSetPoint, MountTorques},
        windloads,
        windloads::{M1Loads, M2Loads, MountLoads},
        Smooth, Weight,
    },
    prelude::*,
    ArcMutex,
};
use fem::{
    dos::{DiscreteModalSolver, ExponentialMatrix},
    fem_io::*,
    FEM,
};
use nalgebra as na;
use parse_monitors::cfd;
use std::{collections::HashMap, env, fs::File, path::Path};
use uid::UID;
use vec_box::vec_box;

fn fig_2_mode(sid: u32) -> na::DMatrix<f64> {
    let root_env = env::var("M1CALIBRATION").unwrap_or_else(|_| ".".to_string());
    let root = Path::new(&root_env);
    let fig_2_mode: Vec<f64> =
        bincode::deserialize_from(File::open(root.join(format!("m1s{sid}fig2mode.bin"))).unwrap())
            .unwrap();
    if sid < 7 {
        na::DMatrix::from_vec(162, 602, fig_2_mode)
    } else {
        na::DMatrix::from_vec(151, 579, fig_2_mode).insert_rows(151, 11, 0f64)
    }
}

#[derive(UID)]
#[alias(
    name = "OSSM1Lcl",
    client = "DiscreteModalSolver<ExponentialMatrix>",
    traits = "Write,Size"
)]
enum M1RigidBodyMotions {}
#[derive(UID)]
#[alias(
    name = "MCM2Lcl6D",
    client = "DiscreteModalSolver<ExponentialMatrix>",
    traits = "Write,Size"
)]
enum M2RigidBodyMotions {}
#[derive(UID)]
#[alias(
    name = "M1SegmentsAxialD",
    client = "DiscreteModalSolver<ExponentialMatrix>",
    traits = "Write"
)]
enum M1BendingModes {}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let sim_sampling_frequency = 1000_usize; // Hz
    let sim_duration = 400_usize; // seconds

    let job_idx = env::var("AWS_BATCH_JOB_ARRAY_INDEX")
        .expect("AWS_BATCH_JOB_ARRAY_INDEX env var missing")
        .parse::<usize>()
        .expect("AWS_BATCH_JOB_ARRAY_INDEX parsing failed");

    let cfd_case = if cfg!(feature = "ze00") {
        env::set_var(
            "FEM_REPO",
            Path::new("/fsx")
                .join("Rodrigo")
                .join("20220611_1945_MT_mount_zen_00_m1HFN_FSM_"),
        );
        cfd::Baseline::<2021>::at_zenith(cfd::ZenithAngle::Zero)
    } else if cfg!(feature = "ze60") {
        env::set_var(
            "FEM_REPO",
            Path::new("/fsx")
                .join("Rodrigo")
                .join("20220621_1443_MT_mount_zen_60_m1HFN_FSM_"),
        );
        cfd::Baseline::<2021>::at_zenith(cfd::ZenithAngle::Sixty)
    } else {
        env::set_var(
            "FEM_REPO",
            Path::new("/fsx")
                .join("Rodrigo")
                .join("20220610_1023_MT_mount_zen_30_m1HFN_FSM_"),
        );
        cfd::Baseline::<2021>::at_zenith(cfd::ZenithAngle::Thirty)
    }
    .into_iter()
    .nth(job_idx)
    .expect(&format!("failed to load CFD case #{}", job_idx));
    let cfd_path = cfd::Baseline::<2021>::path().join(cfd_case.to_string());
    env::set_var("DATA_REPO", &cfd_path);
    log::info!("data repo: {:?}", &cfd_path);

    let mut fem = FEM::from_env()?.static_from_env()?;
    let n_io = (fem.n_inputs(), fem.n_outputs());
    println!("{:}", fem);

    let descriptions = fem.inputs[0]
        .as_ref()
        .map(|i| i.get_by(|x| Some(x.descriptions.clone())))
        .map(|x| {
            x.into_iter()
                .step_by(6)
                .enumerate()
                .map(|(k, x)| format!("{:02}.{}", k, x))
                .collect::<Vec<String>>()
        })
        .map(|x| x.join("\n"));
    log::info!("\n{:}", descriptions.unwrap());

    let cfd_loads = windloads::CfdLoads::foh(cfd_path.to_str().unwrap(), sim_sampling_frequency)
        .duration(sim_duration as f64)
        .mount(&mut fem, 0, None)
        .m1_segments()
        .m2_segments()
        .build()
        .unwrap()
        .into_arcx();

    let state_space = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        //.truncate_hankel_singular_values(1e-4)
        //.max_eigen_frequency(75.)
        .use_static_gain_compensation(n_io)
        .ins::<CFD2021106F>()
        .ins::<OSSElDriveTorque>()
        .ins::<OSSAzDriveTorque>()
        .ins::<OSSRotDriveTorque>()
        .ins::<OSSM1Lcl6F>()
        .ins::<MCM2LclForce6F>()
        .outs::<OSSAzEncoderAngle>()
        .outs::<OSSElEncoderAngle>()
        .outs::<OSSRotEncoderAngle>()
        .outs::<OSSM1Lcl>()
        .outs::<MCM2Lcl6D>()
        .outs_with::<M1Segment1AxialD>(fig_2_mode(1))
        .outs_with::<M1Segment2AxialD>(fig_2_mode(2))
        .outs_with::<M1Segment3AxialD>(fig_2_mode(3))
        .outs_with::<M1Segment4AxialD>(fig_2_mode(4))
        .outs_with::<M1Segment5AxialD>(fig_2_mode(5))
        .outs_with::<M1Segment6AxialD>(fig_2_mode(6))
        .outs_with::<M1Segment7AxialD>(fig_2_mode(7))
        .build()?
        .into_arcx();

    let mut meta_data: HashMap<String, String> = HashMap::new();
    meta_data.insert(
        "sim_sampling_frequency".to_string(),
        format!("{sim_sampling_frequency}"),
    );
    meta_data.insert("sim_duration".to_string(), format!("{sim_duration}"));
    meta_data.insert("FEM".to_string(), env::var("FEM_REPO").unwrap());
    meta_data.insert("CFD_CASE".to_string(), cfd_case.to_string());

    let n_step = sim_duration * sim_sampling_frequency;
    let logging = Arrow::builder(n_step)
        .metadata(meta_data)
        .filename("windloading.parquet")
        .build()
        .into_arcx();

    let mnt_ctrl = Mount::new().into_arcx();

    let mut source: Initiator<_> = Actor::new(cfd_loads.clone());
    let mut sink = Terminator::<_>::new(logging.clone());

    let signal: std::result::Result<OneSignal, _> = Signals::new(1, n_step)
        .output_signal(
            0,
            Signal::Sigmoid {
                amplitude: 1f64,
                sampling_frequency_hz: sim_sampling_frequency as f64,
            },
        )
        .progress()
        .into();
    let mut sigmoid: Initiator<OneSignal, 1> = (signal?, "Sigmoid").into();
    let mut smooth_m1_loads: Actor<_> = Smooth::new().into();
    let mut smooth_m2_loads: Actor<_> = Smooth::new().into();
    let mut smooth_mount_loads: Actor<_> = Smooth::new().into();

    sigmoid
        .add_output()
        .multiplex(3)
        .build::<Weight>()
        .into_input(&mut smooth_m1_loads)
        .into_input(&mut smooth_m2_loads)
        .into_input(&mut smooth_mount_loads)
        .confirm()?;
    source
        .add_output()
        .build::<M1Loads>()
        .into_input(&mut smooth_m1_loads);

    source
        .add_output()
        .build::<M2Loads>()
        .into_input(&mut smooth_m2_loads);

    source
        .add_output()
        .build::<MountLoads>()
        .into_input(&mut smooth_mount_loads);

    // FEM
    let mut fem: Actor<_> = Actor::new(state_space.clone());
    // MOUNT
    let mut mount: Actor<_> = Actor::new(mnt_ctrl.clone());

    smooth_mount_loads
        .add_output()
        .build::<CFD2021106F>()
        .into_input(&mut fem);
    smooth_m1_loads
        .add_output()
        .build::<OSSM1Lcl6F>()
        .into_input(&mut fem);
    smooth_m2_loads
        .add_output()
        .build::<MCM2LclForce6F>()
        .into_input(&mut fem);

    let mut mount_set_point: Initiator<_> = Signals::new(3, n_step).into();
    mount_set_point
        .add_output()
        .build::<MountSetPoint>()
        .into_input(&mut mount);
    mount
        .add_output()
        .build::<MountTorques>()
        .into_input(&mut fem);

    fem.add_output()
        .bootstrap()
        .build::<MountEncoders>()
        .into_input(&mut mount);
    fem.add_output()
        .bootstrap()
        .build::<M1RigidBodyMotions>()
        .logn(&mut sink, 42)
        .await;
    fem.add_output()
        .bootstrap()
        .build::<M2RigidBodyMotions>()
        .logn(&mut sink, 42)
        .await;
    fem.add_output()
        .bootstrap()
        .build::<M1BendingModes>()
        .logn(&mut sink, 162 * 7)
        .await;

    Model::new(vec_box![
        source,
        mount_set_point,
        fem,
        mount,
        sink,
        sigmoid,
        smooth_m1_loads,
        smooth_m2_loads,
        smooth_mount_loads
    ])
    .name("windloading")
    .flowchart()
    .check()?
    .run()
    .wait()
    .await?;

    Ok(())
}
