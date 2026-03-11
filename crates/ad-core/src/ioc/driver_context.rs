use std::sync::Arc;

use crate::ndarray_pool::NDArrayPool;
use crate::plugin::channel::NDArraySender;

/// Abstraction over a detector driver's runtime, providing what plugin
/// configure commands need: an array pool and a way to wire downstream.
pub trait DriverContext: Send + Sync {
    /// The shared NDArrayPool for array allocation.
    fn pool(&self) -> Arc<NDArrayPool>;

    /// Connect a plugin's sender as a downstream consumer of this driver's arrays.
    fn connect_downstream(&self, sender: NDArraySender);
}
