// Auto-generated. Do not edit!

// (in-package uav_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let UAVPath = require('./UAVPath.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ResolvedPaths {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ldr_id = null;
      this.uav_ids = null;
      this.uav_paths = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('ldr_id')) {
        this.ldr_id = initObj.ldr_id
      }
      else {
        this.ldr_id = 0;
      }
      if (initObj.hasOwnProperty('uav_ids')) {
        this.uav_ids = initObj.uav_ids
      }
      else {
        this.uav_ids = [];
      }
      if (initObj.hasOwnProperty('uav_paths')) {
        this.uav_paths = initObj.uav_paths
      }
      else {
        this.uav_paths = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResolvedPaths
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ldr_id]
    bufferOffset = _serializer.int32(obj.ldr_id, buffer, bufferOffset);
    // Serialize message field [uav_ids]
    bufferOffset = _arraySerializer.string(obj.uav_ids, buffer, bufferOffset, null);
    // Serialize message field [uav_paths]
    // Serialize the length for message field [uav_paths]
    bufferOffset = _serializer.uint32(obj.uav_paths.length, buffer, bufferOffset);
    obj.uav_paths.forEach((val) => {
      bufferOffset = UAVPath.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResolvedPaths
    let len;
    let data = new ResolvedPaths(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ldr_id]
    data.ldr_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [uav_ids]
    data.uav_ids = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [uav_paths]
    // Deserialize array length for message field [uav_paths]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.uav_paths = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.uav_paths[i] = UAVPath.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.uav_ids.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.uav_paths.forEach((val) => {
      length += UAVPath.getMessageSize(val);
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uav_control/ResolvedPaths';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '66074861de72e89556e85750791c9c7b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 自定义消息类型：LDR解决后的路径信息
    # 文件路径: uav_control/msg/ResolvedPaths.msg
    
    Header header
    int32 ldr_id
    string[] uav_ids
    UAVPath[] uav_paths
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
    MSG: uav_control/UAVPath
    # 单个无人机的路径信息
    # 文件路径: uav_control/msg/UAVPath.msg
    
    string uav_id
    geometry_msgs/Point[] path_points
    int32[] time_indices
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
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
    const resolved = new ResolvedPaths(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.ldr_id !== undefined) {
      resolved.ldr_id = msg.ldr_id;
    }
    else {
      resolved.ldr_id = 0
    }

    if (msg.uav_ids !== undefined) {
      resolved.uav_ids = msg.uav_ids;
    }
    else {
      resolved.uav_ids = []
    }

    if (msg.uav_paths !== undefined) {
      resolved.uav_paths = new Array(msg.uav_paths.length);
      for (let i = 0; i < resolved.uav_paths.length; ++i) {
        resolved.uav_paths[i] = UAVPath.Resolve(msg.uav_paths[i]);
      }
    }
    else {
      resolved.uav_paths = []
    }

    return resolved;
    }
};

module.exports = ResolvedPaths;
