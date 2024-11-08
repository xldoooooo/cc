// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let mpc_ref_point = require('./mpc_ref_point.js');
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class mpc_ref_traj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.mpc_ref_points = null;
      this.goal = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('mpc_ref_points')) {
        this.mpc_ref_points = initObj.mpc_ref_points
      }
      else {
        this.mpc_ref_points = [];
      }
      if (initObj.hasOwnProperty('goal')) {
        this.goal = initObj.goal
      }
      else {
        this.goal = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mpc_ref_traj
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [mpc_ref_points]
    // Serialize the length for message field [mpc_ref_points]
    bufferOffset = _serializer.uint32(obj.mpc_ref_points.length, buffer, bufferOffset);
    obj.mpc_ref_points.forEach((val) => {
      bufferOffset = mpc_ref_point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [goal]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.goal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mpc_ref_traj
    let len;
    let data = new mpc_ref_traj(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [mpc_ref_points]
    // Deserialize array length for message field [mpc_ref_points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.mpc_ref_points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.mpc_ref_points[i] = mpc_ref_point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [goal]
    data.goal = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.mpc_ref_points.forEach((val) => {
      length += mpc_ref_point.getMessageSize(val);
    });
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/mpc_ref_traj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4faae5b6cb706021d9657ec34fedc421';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    mpc_ref_point[] mpc_ref_points
    geometry_msgs/Vector3 goal
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: quadrotor_msgs/mpc_ref_point
    Header header
    geometry_msgs/Vector3 position
    geometry_msgs/Vector3 velocity
    geometry_msgs/Vector3 acceleration
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mpc_ref_traj(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.mpc_ref_points !== undefined) {
      resolved.mpc_ref_points = new Array(msg.mpc_ref_points.length);
      for (let i = 0; i < resolved.mpc_ref_points.length; ++i) {
        resolved.mpc_ref_points[i] = mpc_ref_point.Resolve(msg.mpc_ref_points[i]);
      }
    }
    else {
      resolved.mpc_ref_points = []
    }

    if (msg.goal !== undefined) {
      resolved.goal = geometry_msgs.msg.Vector3.Resolve(msg.goal)
    }
    else {
      resolved.goal = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = mpc_ref_traj;
