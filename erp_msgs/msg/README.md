# ERP Custom Message

- 230520 kbub chson
- adapted from erp42 package

- ErpData : parses data from erp in this form of message.
- ErpCmd : sends Data using this type of message

## if you want get meaningful data from ERP : 
- contain 'ErpData' message file in your node, and read it using subscriber.

## if you want control ERP : 
- contain 'ErpCmd' message file in your node, and publish data.
