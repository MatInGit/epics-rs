//! Generic passthrough plugin processor.
//!
//! Used as a stub for plugin types that are not yet fully implemented
//! but need to appear in the OPI with correct metadata.

use std::sync::Arc;

use ad_core::ndarray::NDArray;
use ad_core::ndarray_pool::NDArrayPool;
use ad_core::plugin::runtime::NDPluginProcess;

/// A no-op plugin processor that passes arrays through unchanged.
pub struct PassthroughProcessor {
    plugin_type: String,
}

impl PassthroughProcessor {
    pub fn new(plugin_type: &str) -> Self {
        Self {
            plugin_type: plugin_type.to_string(),
        }
    }
}

impl NDPluginProcess for PassthroughProcessor {
    fn plugin_type(&self) -> &str {
        &self.plugin_type
    }

    fn process_array(&mut self, array: &NDArray, _pool: &NDArrayPool) -> Vec<Arc<NDArray>> {
        // Pass through unchanged - create a clone via the pool or just return empty
        // For a passthrough, we don't output anything (acts as a sink)
        // If we wanted true passthrough, we'd clone the array
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_passthrough_plugin_type() {
        let p = PassthroughProcessor::new("NDPluginAttribute");
        assert_eq!(p.plugin_type(), "NDPluginAttribute");
    }
}
