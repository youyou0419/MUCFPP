// Auto-generated. Do not edit!

// (in-package uav_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class UAVPath {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.uav_id = null;
      this.path_points = null;
      this.time_indices = null;
    }
    else {
      if (initObj.hasOwnProperty('uav_id')) {
        this.uav_id = initObj.uav_id
      }
      else {
        this.uav_id = '';
      }
      if (initObj.hasOwnProperty('path_points')) {
        this.path_points = initObj.path_points
      }
      else {
        this.path_points = [];
      }
      if (initObj.hasOwnProperty('time_indices')) {
        this.time_indices = initObj.time_indices
      }
      else {
        this.time_indices = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UAVPath
    // Serialize message field [uav_id]
    bufferOffset = _serializer.string(obj.uav_id, buffer, bufferOffset);
    // Serialize message field [path_points]
    // Serialize the length for message field [path_points]
    bufferOffset = _serializer.uint32(obj.path_points.length, buffer, bufferOffset);
    obj.path_points.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [time_indices]
    bufferOffset = _arraySerializer.int32(obj.time_indices, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UAVPath
    let len;
    let data = new UAVPath(null);
    // Deserialize message field [uav_id]
    data.uav_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [path_points]
    // Deserialize array length for message field [path_points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.path_points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.path_points[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [time_indices]
    data.time_indices = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.uav_id);
    length += 24 * object.path_points.length;
    length += 4 * object.time_indices.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uav_control/UAVPath';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '11d1db57573332d6454c162bb2bcf71b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new UAVPath(null);
    if (msg.uav_id !== undefined) {
      resolved.uav_id = msg.uav_id;
    }
    else {
      resolved.uav_id = ''
    }

    if (msg.path_points !== undefined) {
      resolved.path_points = new Array(msg.path_points.length);
      for (let i = 0; i < resolved.path_points.length; ++i) {
        resolved.path_points[i] = geometry_msgs.msg.Point.Resolve(msg.path_points[i]);
      }
    }
    else {
      resolved.path_points = []
    }

    if (msg.time_indices !== undefined) {
      resolved.time_indices = msg.time_indices;
    }
    else {
      resolved.time_indices = []
    }

    return resolved;
    }
};

module.exports = UAVPath;
