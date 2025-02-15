baremodule ProtocolCode

using Base: @enum

@enum ProtocolCodeEnum begin

    # BASIC
    HEADER = 0xFE
    FOOTER = 0xFA

    # MDI MODE AND OPERATION
    GET_ANGLES = 0x20

    # ATOM IO
    GET_DIGITAL_INPUT = 0x62

    # Basic
    GET_BASIC_INPUT = 0xA1

end

end

import .ProtocolCode: ProtocolCodeEnum
