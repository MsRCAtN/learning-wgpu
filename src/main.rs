use app_surface::{AppSurface, SurfaceFrame};
use std::iter;
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;
use winit::{
    dpi::PhysicalSize,
    event::*,
    event_loop::{EventLoop, EventLoopWindowTarget},
    keyboard::{Key, NamedKey},
    window::{WindowBuilder, WindowId},
};
use wgpu::util::DeviceExt;
struct State {
    app: AppSurface,
    render_pipeline: wgpu::RenderPipeline,
    challenge_render_pipeline: wgpu::RenderPipeline,
    use_color: bool,
    vertex_buffer: wgpu::Buffer,
}

pub trait Action {
    fn new(app: app_surface::AppSurface) -> Self;
    fn get_adapter_info(&self) -> wgpu::AdapterInfo;
    fn current_window_id(&self) -> WindowId;
    fn start(&mut self);
    fn resize(&mut self, size: &PhysicalSize<u32>);
    fn request_redraw(&mut self);
    fn input(&mut self, _event: &WindowEvent) -> bool {
        false
    }
    fn update(&mut self) {}
    fn render(&mut self) -> Result<(), wgpu::SurfaceError>;
}

impl Action for State {
    fn new(app: app_surface::AppSurface) -> Self {
        let shader = app
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("Shader"),
                source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
            });
        // new()
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Vertex Buffer"),
            contents: bytemuck::cast_slice(VERTICES),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let render_pipeline_layout =
            app.device
                .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("Render PipeLine Layout"),
                    bind_group_layouts: &[],
                    push_constant_ranges: &[],
                });
        let render_pipeline = app
            .device
            .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("Render Pipeline"),
                layout: Some(&render_pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: "vs_main",
                    compilation_options: Default::default(),
                    buffers: &[],
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: "fs_main",
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: app.config.format.add_srgb_suffix(),
                        blend: Some(wgpu::BlendState {
                            color: wgpu::BlendComponent::REPLACE,
                            alpha: wgpu::BlendComponent::REPLACE,
                        }),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                }),
                primitive: wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::TriangleList,
                    strip_index_format: None,
                    front_face: wgpu::FrontFace::Ccw,
                    cull_mode: Some(wgpu::Face::Back),
                    polygon_mode: wgpu::PolygonMode::Fill,
                    // Requires Features::DEPTH_CLIP_CONTROL
                    unclipped_depth: false,
                    // Requires Features::CONSERVATIVE_RASTERIZATION
                    conservative: false,
                },
                depth_stencil: None,
                multisample: wgpu::MultisampleState {
                    count: 1,
                    mask: !0,
                    alpha_to_coverage_enabled: false,
                },
                // If the pipeline will be used with a multiview render pass, this
                // indicates how many array layers the attachments will have.
                multiview: None,
            });
        let shader = app
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("Challenge Shader"),
                source: wgpu::ShaderSource::Wgsl(include_str!("challenge.wgsl").into()),
            });

        let challenge_render_pipeline =
            app.device
                .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("Render Pipeline"),
                    layout: Some(&render_pipeline_layout),
                    vertex: wgpu::VertexState {
                        module: &shader,
                        entry_point: "vs_main",
                        compilation_options: Default::default(),
                        buffers: &[],
                    },
                    fragment: Some(wgpu::FragmentState {
                        module: &shader,
                        entry_point: "fs_main",
                        compilation_options: Default::default(),
                        targets: &[Some(wgpu::ColorTargetState {
                            format: app.config.format.add_srgb_suffix(),
                            blend: Some(wgpu::BlendState::REPLACE),
                            write_mask: wgpu::ColorWrites::ALL,
                        })],
                    }),
                    primitive: wgpu::PrimitiveState {
                        topology: wgpu::PrimitiveTopology::TriangleList,
                        strip_index_format: None,
                        front_face: wgpu::FrontFace::Ccw,
                        cull_mode: Some(wgpu::Face::Back),
                        // Setting this to anything other than Fill requires Features::NON_FILL_POLYGON_MODE
                        polygon_mode: wgpu::PolygonMode::Fill,
                        ..Default::default()
                    },
                    depth_stencil: None,
                    multisample: wgpu::MultisampleState {
                        count: 1,
                        mask: !0,
                        alpha_to_coverage_enabled: false,
                    },
                    // If the pipeline will be used with a multiview render pass, this
                    // indicates how many array layers the attachments will have.
                    multiview: None,
                });

        let use_color = true;

        Self {
            app,
            render_pipeline,
            challenge_render_pipeline,
            use_color,
        }
    }
    fn start(&mut self) {
        //  只有在进入事件循环之后，才有可能真正获取到窗口大小。
        let size = self.app.get_view().inner_size();
        self.resize(&size);
    }
    fn get_adapter_info(&self) -> wgpu::AdapterInfo {
        self.app.adapter.get_info()
    }
    fn current_window_id(&self) -> WindowId {
        self.app.get_view().id()
    }

    fn resize(&mut self, size: &PhysicalSize<u32>) {
        if self.app.config.width == size.width && self.app.config.height == size.height {
            return;
        }
        self.app.resize_surface();
    }
    fn request_redraw(&mut self) {
        self.app.get_view().request_redraw();
    }

    fn input(&mut self, event: &WindowEvent) -> bool {
        match event {
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        state,
                        logical_key: Key::Named(NamedKey::Space),
                        ..
                    },
                ..
            } => {
                self.use_color = *state == ElementState::Released;
                true
            }
            _ => false,
        }
    }

    fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        let (output, view) = self.app.get_current_frame_view(None);

        let mut encoder = self
            .app
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            // Background Color
                            r: 0.2,
                            g: 0.0,
                            b: 0.0,
                            a: 0.0, // alpha channel
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                ..Default::default()
            });

            render_pass.set_pipeline(if self.use_color {
                &self.render_pipeline
            } else {
                &self.challenge_render_pipeline
            });
            render_pass.draw(0..3, 0..1);
        }

        self.app.queue.submit(iter::once(encoder.finish()));
        output.present();

        Ok(())
    }
}

/* impl State {

    async fn new(window: Arc<Window>) -> Self {
        // The instance is a handle to our GPU
        // BackendBit::PRIMARY => Vulkan + Metal + DX12 + Browser WebGPU
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });
        let surface = instance.create_surface(window.clone()).unwrap();

        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::default(),
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .unwrap();

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: None,
                    required_features: wgpu::Features::empty(),
                    // WebGL doesn't support all of wgpu's features, so if
                    // we're building for the web we'll have to disable some.
                    required_limits: if cfg!(target_arch = "wasm32") {
                        wgpu::Limits::downlevel_webgl2_defaults()
                    } else {
                        wgpu::Limits::default()
                    },
                },
                // Some(&std::path::Path::new("trace")), // Trace path
                None,
            )
            .await
            .unwrap();

        let mut size = window.inner_size();
        size.width = size.width.max(1);
        size.height = size.height.max(1);
        let config = surface
            .get_default_config(&adapter, size.width, size.height)
            .unwrap();
        surface.configure(&device, &config);
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });
        let render_pipeline_layout =
        device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("Render Pipeline Layout"),
        bind_group_layouts: &[],
        push_constant_ranges: &[],
        });
        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Render Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                compilation_options: Default::default(),
                entry_point: "vs_main", // 1.
                buffers: &[], // 2.
            },
            fragment: Some(wgpu::FragmentState { // 3.
                module: &shader,
                compilation_options: Default::default(),
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState { // 4.
                    format: config.format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList, // 1.
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw, // 2.
                cull_mode: Some(wgpu::Face::Back),
                // 将此设置为 Fill 以外的任何值都要需要开启 Feature::NON_FILL_POLYGON_MODE
                polygon_mode: wgpu::PolygonMode::Fill,
                // 需要开启 Features::DEPTH_CLIP_CONTROL
                unclipped_depth: false,
                // 需要开启 Features::CONSERVATIVE_RASTERIZATION
                conservative: false,
            },
            depth_stencil: None, // 1.
            multisample: wgpu::MultisampleState {
                count: 1, // 2.
                mask: !0, // 3.
                alpha_to_coverage_enabled: false, // 4.
            },
            multiview: None, // 5.
        });
        Self {
            window,
            surface,
            _adapter: adapter,
            device,
            queue,
            config,
            size,
            render_pipeline,
        }
    }

    pub fn start(&mut self) {
        //  只有在进入事件循环之后，才有可能真正获取到窗口大小。
        let size = self.window.inner_size();
        self.resize(size);
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        if new_size.width > 0 && new_size.height > 0 {
            self.size = new_size;
            self.config.width = new_size.width;
            self.config.height = new_size.height;
            self.surface.configure(&self.device, &self.config);
        }
    }

    fn update(&mut self) {}

    fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        if self.size.width == 0 || self.size.height == 0 {
            return Ok(());
        }
        let output = self.surface.get_current_texture()?;
        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.2,
                            b: 0.3,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                ..Default::default()
            });
            render_pass.set_pipeline(&self.render_pipeline); // 2.
            render_pass.draw(0..3, 0..1); // 3.
        }

        self.queue.submit(iter::once(encoder.finish()));
        output.present();

        Ok(())
    }
}
 */

fn start_event_loop<A: Action + 'static>(event_loop: EventLoop<()>, instance: A) {
    let mut state = instance;
    let mut last_render_time = Instant::now();
    cfg_if::cfg_if! {
        if #[cfg(target_arch = "wasm32")] {
            use winit::platform::web::EventLoopExtWebSys;
            let event_loop_function = EventLoop::spawn;
        } else {
            let event_loop_function = EventLoop::run;
        }
    }
    let _ = (event_loop_function)(
        event_loop,
        move |event: Event<()>, elwt: &EventLoopWindowTarget<()>| {
            if event == Event::NewEvents(StartCause::Init) {
                state.start();
            }
            if let Event::WindowEvent { event, .. } = event {
                if !state.input(&event) {
                    match event {
                        WindowEvent::KeyboardInput {
                            event:
                                KeyEvent {
                                    logical_key: Key::Named(NamedKey::Escape),
                                    ..
                                },
                            ..
                        }
                        | WindowEvent::CloseRequested => elwt.exit(),
                        WindowEvent::Resized(physical_size) => {
                            if physical_size.width == 0 || physical_size.height == 0 {
                                // 处理最小化窗口的事件
                                println!("Window minimized!");
                            } else {
                                state.resize(&physical_size);
                            }
                        }
                        WindowEvent::RedrawRequested => {
                            let now = Instant::now();
                            let _dt = now - last_render_time;
                            last_render_time = now;
                            state.update();

                            match state.render() {
                                Ok(_) => {}
                                // 当展示平面的上下文丢失，就需重新配置
                                Err(wgpu::SurfaceError::Lost) => eprintln!("Surface is lost"),
                                // 所有其他错误（过期、超时等）应在下一帧解决
                                Err(e) => eprintln!("SurfaceError: {e:?}"),
                            }
                            // 除非我们手动请求，RedrawRequested 将只会触发一次。
                            state.request_redraw();
                        }
                        _ => {}
                    }
                }
            }
        },
    );
}
async fn create_action_instance<A: Action + 'static>(
    wh_ratio: Option<f32>,
    #[cfg(target_arch = "wasm32")] html_canvas_container_id: Option<&'static str>,
) -> (EventLoop<()>, A) {
    let event_loop = EventLoop::new().unwrap();
    let window = WindowBuilder::new().build(&event_loop).unwrap();
    let scale_factor = window.scale_factor() as f32;

    // 计算一个默认显示高度
    let height = (if cfg!(target_arch = "wasm32") {
        500.0
    } else {
        600.0
    } * scale_factor) as u32;

    let width = if let Some(ratio) = wh_ratio {
        (height as f32 * ratio) as u32
    } else {
        height
    };
    if cfg!(not(target_arch = "wasm32")) {
        let _ = window.request_inner_size(PhysicalSize::new(width, height));
    }

    #[cfg(target_arch = "wasm32")]
    {
        // Winit prevents sizing with CSS, so we have to set
        // the size manually when on web.
        use winit::platform::web::WindowExtWebSys;
        web_sys::window()
            .and_then(|win| win.document())
            .map(|doc| {
                let element_id = if let Some(container_id) = html_canvas_container_id {
                    container_id
                } else {
                    "wasm-example"
                };
                let canvas = window.canvas().unwrap();
                let mut web_width = 800.0f32;
                let ratio = if let Some(ratio) = wh_ratio {
                    ratio
                } else {
                    1.0
                };

                match doc.get_element_by_id(&element_id) {
                    Some(dst) => {
                        web_width = dst.client_width() as f32;
                        let _ = dst.append_child(&web_sys::Element::from(canvas)).ok();
                    }
                    None => {
                        canvas.style().set_css_text(
                            &(canvas.style().css_text()
                                + "background-color: black; display: block; margin: 20px auto;"),
                        );
                        doc.body()
                            .map(|body| body.append_child(&web_sys::Element::from(canvas)).ok());
                    }
                };
                // winit 0.29 开始，通过 request_inner_size, canvas.set_width 都无法设置 canvas 的大小
                let canvas = window.canvas().unwrap();
                let web_height = web_width / ratio;
                let scale_factor = window.scale_factor() as f32;
                canvas.set_width((web_width * scale_factor) as u32);
                canvas.set_height((web_height * scale_factor) as u32);
                canvas.style().set_css_text(
                    &(canvas.style().css_text()
                        + &format!("width: {}px; height: {}px", web_width, web_height)),
                );
            })
            .expect("Couldn't append canvas to document body.");
    };

    let app = app_surface::AppSurface::new(window).await;
    let instance = A::new(app);

    let adapter_info = instance.get_adapter_info();
    let gpu_info = format!(
        "正在使用 {}, 后端图形接口为 {:?}。",
        adapter_info.name, adapter_info.backend
    );
    #[cfg(not(target_arch = "wasm32"))]
    println!("{gpu_info}");
    #[cfg(target_arch = "wasm32")]
    log::warn!(
        "{gpu_info:?}\n这不是一条警告，仅仅是为了在控制台能默认打印出来而不必开启 info 日志等级。"
    );

    (event_loop, instance)
}

#[cfg(not(target_arch = "wasm32"))]
pub fn run<A: Action + 'static>(
    wh_ratio: Option<f32>,
    _html_canvas_container_id: Option<&'static str>,
) {
    env_logger::init();

    let (event_loop, instance) = pollster::block_on(create_action_instance::<A>(wh_ratio));
    start_event_loop::<A>(event_loop, instance);
}

#[cfg(target_arch = "wasm32")]
pub fn run<A: Action + 'static>(
    wh_ratio: Option<f32>,
    html_canvas_container_id: Option<&'static str>,
) {
    use wasm_bindgen::prelude::*;

    std::panic::set_hook(Box::new(console_error_panic_hook::hook));
    console_log::init_with_level(log::Level::Warn).expect("无法初始化日志库");

    wasm_bindgen_futures::spawn_local(async move {
        let (event_loop, instance) =
            create_action_instance::<A>(wh_ratio, html_canvas_container_id).await;
        let run_closure =
            Closure::once_into_js(move || start_event_loop::<A>(event_loop, instance));

        // 处理运行过程中抛出的 JS 异常。
        // 否则 wasm_bindgen_futures 队列将中断，且不再处理任何任务。
        if let Err(error) = call_catch(&run_closure) {
            let is_control_flow_exception = error.dyn_ref::<js_sys::Error>().map_or(false, |e| {
                e.message().includes("Using exceptions for control flow", 0)
            });

            if !is_control_flow_exception {
                web_sys::console::error_1(&error);
            }
        }

        #[wasm_bindgen]
        extern "C" {
            #[wasm_bindgen(catch, js_namespace = Function, js_name = "prototype.call.call")]
            fn call_catch(this: &JsValue) -> Result<(), JsValue>;
        }
    });
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
struct Vertex {
    position: [f32; 3],
    color: [f32; 3],
}
const VERTICES: &[Vertex] = &[
    Vertex {
        position: [0.0, 0.5, 0.0],
        color: [1.0, 0.0, 0.0],
    },
    Vertex {
        position: [-0.5, -0.5, 0.0],
        color: [0.0, 1.0, 0.0],
    },
    Vertex {
        position: [0.5, -0.5, 0.0],
        color: [0.0, 0.0, 1.0],
    },
];

impl Vertex {
    fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x3,
                },
            ],
        }
    }
}

fn main() {
    run::<State>(None, None);
}
