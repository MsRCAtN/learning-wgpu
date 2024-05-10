// 顶点着色器

struct VertexOutput {
    @builtin(position) clip_position: vec4f,
};

@vertex
fn vs_main(
    @builtin(vertex_index) in_vertex_index: u32,
) -> VertexOutput {
    var out: VertexOutput;
    let x = f32(1 - i32(in_vertex_index)) * 1;
    let y = f32(i32(in_vertex_index & 1u) * 2 - 1) * 0.5; // in_vertex_index 为奇数时 y = 0.5， 偶数时 y = -0.5
    out.clip_position = vec4f(x, y, 0.2, 1);  //  Clip coordinates (Xc, Yc, Zc, Wc) : (x, y, z) = (Xc/Wc, Yc/Wc, Zc/Wc)
    return out;
}

// 片元着色器

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4f {
    return vec4f(1, 1, 1, 1);
}