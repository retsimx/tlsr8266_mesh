#[no_mangle]
fn sqrtf(val: f32) -> f32 {
    return micromath::F32Ext::sqrt(val);
}