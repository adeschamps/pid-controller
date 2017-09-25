use std::ops::{Add, Sub, Mul, Div, AddAssign};
use std::marker::PhantomData;

// Control = P * Measure
// P = Control / Measure

// Control = I * Measure * Time
// I = Control / (Measure * Time)

// Control = D * (Measure / Time)
// D = Control * Time / Measure

/// A simple PID controller.
///
/// A controller is generic over three parameters:
///
/// - `Measure`: the type of the input, commonly called the set point.
/// - `Control`: the type of the output.
/// - `Time`: the duration between updates.
pub trait PidController<Measure, Control, Time> {
    fn update(&mut self, error: Measure, elapsed: Time);
    fn output(&self) -> Control;
}

pub struct Controller<
    Measure,
    Control,
    Time,
    Integral = <Measure as Mul<Time>>::Output,
    Derivative = <Measure as Div<Time>>::Output,
    P = <Control as Div<Measure>>::Output,
    I = <Control as Div<Integral>>::Output,
    D = <Control as Div<Derivative>>::Output,
> {
    k_p: P,
    k_i: I,
    k_d: D,
    output: Control,
    previous_error: Measure,
    accumulated_error: Integral,
    phantom_time: PhantomData<Time>,
    phantom_derivative: PhantomData<Derivative>,
}


impl <Measure, Control, Time, P, I, D, Integral, Derivative>
    Controller<Measure, Control, Time>
    where Measure: Mul<Time, Output=Integral> + Div<Time, Output=Derivative>,
          Control: Add<Control, Output=Control>
    + Div<Measure, Output=P>
    + Div<Integral, Output=I>
    + Div<Derivative, Output=D>,
{
    pub fn new(k_p: P, k_i: I, k_d: D,
               initial_output: Control,
               initial_error: Measure,
               initial_accumulated_error: Integral) -> Controller<Measure, Control, Time> {
        Controller {
            k_p,
            k_i,
            k_d,
            output: initial_output,
            previous_error: initial_error,
            accumulated_error: initial_accumulated_error,
            phantom_time: PhantomData::<Time>,
            phantom_derivative: PhantomData::<Derivative>,
        }
    }
}


impl<Measure, Control, Time, P, I, D, Integral, Derivative> PidController<Measure, Control, Time>
    for Controller<Measure, Control, Time>
where
    Time: Div<Measure> + Copy,
    Measure: AddAssign<Measure>
        + Sub<Measure, Output = Measure>
        + Mul<Time, Output = Integral>
        + Div<Time, Output = Derivative>
        + Copy,
    Control: Add<Control, Output = Control>
        + Copy
        + Div<Measure, Output = P>
        + Div<Integral, Output = I>
        + Div<Derivative, Output = D>,
    P: Mul<Measure, Output = Control> + Copy,
    I: Mul<Integral, Output = Control> + Copy,
    D: Mul<<Measure as Div<Time>>::Output, Output = Control> + Copy,
    Integral: AddAssign<Integral> + Copy,
{
    fn update(&mut self, error: Measure, delta: Time) {
        self.accumulated_error += error * delta;
        let error_delta = (error - self.previous_error) / delta;
        self.output = self.k_p * error + self.k_i * self.accumulated_error + self.k_d * error_delta;
        self.previous_error = error;
    }

    fn output(&self) -> Control {
        self.output
    }
}

#[cfg(test)]
mod tests {
    extern crate dimensioned;
    use self::dimensioned::si;

    use super::*;

    #[test]
    fn no_integral_gain() {
        let mut controller = Controller::new(1.0, 0.0, 0.0, 0., 0., 0.);

        controller.update(1.0, 1.0);
        let output_a = controller.output();

        controller.update(1.0, 1.0);
        let output_b = controller.output();

        assert_eq!(output_a, output_b);
    }

    #[test]
    fn with_integral_gain() {
        let mut controller = Controller::new(1.0, 0.5, 0.0, 0.0, 0.0, 0.0);

        controller.update(1.0, 1.0);
        let output_a = controller.output();

        controller.update(1.0, 1.0);
        let output_b = controller.output();

        assert!(output_a < output_b);
    }

    #[test]
    fn with_units() {
        let mut controller: Controller<si::Meter<f64>, si::Joule<f64>, si::Second<f64>> =
            Controller::new(
                1.0 * si::J / si::M,
                0.2 * si::J / (si::M * si::S),
                0.5 * si::J / (si::M / si::S),
                0.0 * si::J,
                0.0 * si::M,
                0.0 * si::M * si::S,
            );

        controller.update(1.0 * si::M, 1.0 * si::S);
    }
}
