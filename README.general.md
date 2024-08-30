# data driven system identification
## input:

    log_2023_09_06-13_17_50-openloop.log
## output:
    

    A Good:
        [[ 2.82975412e-15  1.00000000e+00]
         [-9.80269531e-01  1.97873008e+00]]

    B Good:
        [[1.84982170e-15]
         [6.72172757e-03]]

K from dLQR (with Q = [[1,0],[0,1]] = eye(2) and R = [[1]] = eye(1)):

    K: 
        [[ 15.12857803 -16.25584753]]

# troubleshooting

## cannot create can controller

solution: https://gitlab.com/lely_industries/lely-core/-/issues/49
```shell
CAN=<can interface name>
BITRATE=<your can bitrate>
ip link set ${CAN} down
ip link set ${CAN} type can bitrate ${BITRATE}
ip link set ${CAN} txqueuelen 1000 #this is the solution :)
ip link set ${CAN} up
```