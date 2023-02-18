use fixed::{FixedI32, FixedU32};
use fixed::types::extra::{U15, U16};

pub struct CubicInt;

impl CubicInt {
    pub fn ease_in(t: u16, b: u16, c: u16, d: u16) -> u16 {
        let t = FixedU32::<U16>::from_num(t);
        let b = FixedU32::<U16>::from_num(b);
        let c = FixedU32::<U16>::from_num(c);
        let d = FixedU32::<U16>::from_num(d);

        // blinken();

        let t = t / d;
        (c * (t * t * t) + b).to_num()
    }

    pub fn ease_out(t: u16, b: u16, c: u16, d: u16) -> u16 {
        let t = FixedI32::<U15>::from_num(t);
        let b = FixedI32::<U15>::from_num(b);
        let c = FixedI32::<U15>::from_num(c);
        let d = FixedI32::<U15>::from_num(d);

        let t = t / d - FixedI32::<U15>::from_num(1);
        (c * ((t * t * t) + FixedI32::<U15>::from_num(1)) + b).to_num()
    }

    pub fn ease_in_out(t: u16, b: FixedI32::<U15>, c: FixedI32::<U15>, d: u16) -> FixedI32::<U15> {
        let t = FixedI32::<U15>::from_num(t);
        let d = FixedI32::<U15>::from_num(d);

        let t = t / (d / 2);
        if t < 1 {
            c / 2 * (t * t * t) + b
        }
        else {
            let t = t - FixedI32::<U15>::from_num(2);
            c / 2 * (t * t * t + FixedI32::<U15>::from_num(2)) + b
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn ease_in() {
        assert_eq!(CubicInt::ease_in(1 * 10000, 2 * 10000, 3 * 10000, 4 * 10000), 20468);
    }

    #[test]
    fn ease_out() {
        assert_eq!(CubicInt::ease_out(1 * 10000, 2 * 10000, 3 * 10000, 4 * 10000), 37343);
    }

    #[test]
    fn ease_in_out() {
        assert_eq!(CubicInt::ease_in_out(1 * 10000, FixedI32::<U15>::from_num(2 * 10000), FixedI32::<U15>::from_num(3 * 10000), 4 * 10000), 21875);
        assert_eq!(CubicInt::ease_in_out(51, FixedI32::<U15>::from_num(1), FixedI32::<U15>::from_num(100), 100), 53);
    }
}