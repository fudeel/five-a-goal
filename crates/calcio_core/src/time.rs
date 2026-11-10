//! Fixed-timestep clock: turns variable frame times into a whole number of
//! physics steps, so the simulation is independent of the frame rate.

/// Accumulates real frame time and hands out fixed physics steps.
#[derive(Debug, Clone)]
pub struct FixedClock {
    dt: f32,
    accumulator: f32,
    /// Frame times above this are clamped (debugger pauses, window drags).
    pub max_frame_time: f32,
    /// Upper bound of steps per frame, to avoid a "spiral of death".
    pub max_steps: u32,
}

impl FixedClock {
    pub fn new(dt: f32) -> Self {
        Self {
            dt,
            accumulator: 0.0,
            max_frame_time: 0.1,
            max_steps: 48,
        }
    }

    /// Fixed timestep length (s).
    pub fn dt(&self) -> f32 {
        self.dt
    }

    /// Adds `frame_time` and returns how many fixed steps to run now.
    pub fn advance(&mut self, frame_time: f32) -> u32 {
        let ft = if frame_time.is_finite() {
            frame_time.clamp(0.0, self.max_frame_time)
        } else {
            0.0
        };
        self.accumulator += ft;
        let mut steps = 0;
        while self.accumulator >= self.dt && steps < self.max_steps {
            self.accumulator -= self.dt;
            steps += 1;
        }
        if steps == self.max_steps {
            // Drop the backlog rather than trying to catch up forever.
            self.accumulator = self.accumulator.min(self.dt);
        }
        steps
    }

    /// Fraction (0..1) of a step left in the accumulator, for interpolation.
    pub fn alpha(&self) -> f32 {
        (self.accumulator / self.dt).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steps_accumulate() {
        let mut c = FixedClock::new(0.01);
        assert_eq!(c.advance(0.005), 0);
        assert_eq!(c.advance(0.006), 1);
        assert_eq!(c.advance(0.0301), 3);
    }

    #[test]
    fn clamps_huge_frames() {
        let mut c = FixedClock::new(1.0 / 240.0);
        let steps = c.advance(5.0);
        assert!(steps <= c.max_steps);
        assert!(steps as f32 * c.dt() <= c.max_frame_time + 1e-4);
    }

    #[test]
    fn ignores_nan() {
        let mut c = FixedClock::new(0.01);
        assert_eq!(c.advance(f32::NAN), 0);
    }
}
