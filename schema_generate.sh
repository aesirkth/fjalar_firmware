#!/bin/sh
nanopb_generator -D application/common/protocol/ schema.proto -I . -C
protoc --python_out=dashboard/ nanopb.proto schema.proto -I .