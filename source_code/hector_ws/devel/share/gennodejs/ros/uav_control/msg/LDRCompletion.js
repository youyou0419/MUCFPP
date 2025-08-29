// Auto-generated. Do not edit!

// (in-package uav_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class LDRCompletion {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ldr_id = null;
      this.uav_ids = null;
      this.message = null;
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
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LDRCompletion
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ldr_id]
    bufferOffset = _serializer.int32(obj.ldr_id, buffer, bufferOffset);
    // Serialize message field [uav_ids]
    bufferOffset = _arraySerializer.string(obj.uav_ids, buffer, bufferOffset, null);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LDRCompletion
    let len;
    let data = new LDRCompletion(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ldr_id]
    data.ldr_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [uav_ids]
    data.uav_ids = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.uav_ids.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += _getByteLength(object.message);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uav_control/LDRCompletion';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b6ac87e9767065ecacf77bb14cf02fd4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # LDR完成广播消息
    Header header
    int32 ldr_id
    string[] uav_ids
    string message
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LDRCompletion(null);
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

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = LDRCompletion;
