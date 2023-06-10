struct VertexOutput {
    @builtin(position) clip_position:vec4<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) in_vertex_index: u32) -> @builtin(position) vec4<f32>{
    return vec4<f32>((vertex_index << 1u) & 2.0, vertex_index & 2.0, 0.0, 1.0);
}

struct Palette {
    colors : array<f32, 256>;
}

struct PixelBuffer {
    pixels : array<u8, 61440>;
}

@fragment
fn fs_main(@builtin(position) frag_coord: vec4<f32>) -> @location(0) vec4<f32>{
    var index = (i32(frag_coord) << 9) | i32(frag_coord.y);
    var pixel = pixel_buffer.pixels[index];
    return palette.colors[pixel];
}
