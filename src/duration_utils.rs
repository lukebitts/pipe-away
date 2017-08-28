use num::Float;

pub trait DurationUtils {
    fn as_delta32(&self) -> f32;
    fn as_delta64(&self) -> f64;
}

impl DurationUtils for ::std::time::Duration {
    fn as_delta32(&self) -> f32 {
        self.as_delta64() as f32
    }
    fn as_delta64(&self) -> f64 {
        (self.as_secs() * 1_000_000_000 + self.subsec_nanos() as u64) as f64 / 1_000_000_000f64
    }
}
