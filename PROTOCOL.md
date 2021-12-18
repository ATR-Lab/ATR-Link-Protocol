# Message format


```
<message>
    <header>
        <sender>
            ATR-Scratch-Server
        </sender>
        <receiver>
            ATR-Wearable-1
        </receiver>
        <timestamp>
            112345667
        </timestamp>
        <sub_device_id>
            1
        </sub_device_id>
        <message_type>
            cmd
        </message_type>
    </header>
    <cmd>
        <opcode> 
            CMD_IMU
        </opcode>
    </cmd>
</message>

```


# Codified

```
0 = message
1 = header
2 = sender
3 = receiver
4 = timestamp
5 = sub_device_id
6 = message_type
7 = command
8 = opcode
```
