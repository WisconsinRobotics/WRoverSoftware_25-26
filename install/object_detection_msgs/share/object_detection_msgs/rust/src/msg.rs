#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to object_detection_msgs__msg__Detection
/// Normalized bounding box coordinates (0.0 to 1.0)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Detection {

    // This member is not documented.
    #[allow(missing_docs)]
    pub x1: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y1: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x2: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y2: f64,

    /// Confidence level (0.0 to 1.0)
    pub conf: f64,

    /// Distance in meters
    pub distance: f64,

}



impl Default for Detection {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Detection::default())
  }
}

impl rosidl_runtime_rs::Message for Detection {
  type RmwMsg = super::msg::rmw::Detection;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        x1: msg.x1,
        y1: msg.y1,
        x2: msg.x2,
        y2: msg.y2,
        conf: msg.conf,
        distance: msg.distance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      x1: msg.x1,
      y1: msg.y1,
      x2: msg.x2,
      y2: msg.y2,
      conf: msg.conf,
      distance: msg.distance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      x1: msg.x1,
      y1: msg.y1,
      x2: msg.x2,
      y2: msg.y2,
      conf: msg.conf,
      distance: msg.distance,
    }
  }
}


