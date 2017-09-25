# Type safe PID controller

This is a simple [PID controller][pid], written in Rust.
Instead of using a concrete numeric type such as `f64`,
all the types it uses are generic.

It turns out that even for a simple calculation such as this,
specifying which operations are valid can get quite verbose.

[pid]: https://en.wikipedia.org/wiki/PID_controller
