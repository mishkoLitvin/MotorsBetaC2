#ifndef SCIDATA_H
#define SCIDATA_H

#define DEV_PWR     0b000
#define DEV_MOTOR   0b001
#define DEV_ANGLE   0b010
#define DEV_VEL     0b011
#define DEV_LOCKDEV 0b100

struct SAAD_CTRL_BITS_{
    Uint8 CONTR:3;
    Uint8 AVTO:1;
    Uint8 ANGLE:1;
    Uint8 SCAN:1;
    Uint8 LOCK_DEV:1;
    Uint8 WORK:1;
}SAAD_CTRL_BITS;

union SAAD_CTRL_{
    Uint8 all;
    struct SAAD_CTRL_BITS_ bit;
}SAAD_CTRL;

struct SAAD_CTRL_ALL_{
	union SAAD_CTRL_ CTRL;
	Uint8 POWER:1;
}SAAD_CTRL_ALL;

struct SAAD_ST_BITS_{
    Uint8 ST_CONTR:1;
    Uint8 reserved:2;
    Uint8 ST_AVTO:1;
    Uint8 ST_ANGLE:1;
    Uint8 ST_SCAN:1;
    Uint8 ST_LOCK_DEV:1;
    Uint8 ST_WORK:1;
}SAAD_ST_BITS;

union SAAD_ST_{
	Uint8 all;
    struct SAAD_ST_BITS_ bit;
}SAAD_ST;

struct SAAD_TESTS_BITS_{
//    Uint8 RESULT:3;
	Uint8 CTRL_PWR:1;
	Uint8 CTRL_MOTOR:1;
	Uint8 CTRL_ANGLE:1;
	Uint8 CTRL_VEL:1;
	Uint8 CTRL_LOCKDEV:1;
	Uint8 reserved:3;
}SAAD_TESTS_BITS;

typedef union SAAD_TESTS_{
	Uint8 all;
    struct SAAD_TESTS_BITS_ bit;
}SAAD_TESTS;

struct SAAD_TEST_RES_BITS_{
	Uint8 ERROR_CODE:3;
	Uint8 reserved:1;
	Uint8 DEV_CODE:3;
	Uint8 reserved0:1;
}SAAD_TEST_RES_BITS;

union SAAD_TEST_RES_{
	Uint8 all;
    struct SAAD_TEST_RES_BITS_ bit;
}SAAD_TEST_RES;

struct COMMAND_BYTE_BITS_{
	Uint8 COMMAND_L:8;
	Uint8 COMMAND_H:8;

}COMMAND_BYTE_BITS;

union COMMAND_BYTE_{
	Uint16 all;
	struct COMMAND_BYTE_BITS_ bit;
}COMMAND_BYTE;

struct VELOCITY_BITS_{
	Uint8 VELOCITY_L:8;
	Uint8 VELOCITY_H:8;
}VELOCITY_BITS;

union VELOCITY_{
	short all;
	struct VELOCITY_BITS_ bit;
}VELOCITY;

struct POSITION_BITS_{
	Uint8 POSITION_L:8;
	Uint8 POSITION_H:8;
}POSITION_BITS;

union POSITION_{
	short all;
	struct POSITION_BITS_ bit;
}POSITION;

struct DATA_BITS_{
	Uint8 DATA_L:8;
	Uint8 DATA_H:8;
}DATA_BITS;

union DATA_{
	short all;
	struct DATA_BITS_ bit;
}DATA;

struct CTRLSUM_BITS_{
	Uint8 CTRLSUM_L:8;
	Uint8 CTRLSUM_H:8;
}CTRLSUM_BITS;

union CTRLSUM_{
	Uint16 all;
	struct CTRLSUM_BITS_ bit;
}CTRLSUM;

struct START_BIT_BITS_{
	Uint8 START_0:8;
	Uint8 START_1:8;
	Uint8 START_2:8;
	Uint8 START_3:8;
}START_BIT_BITS;

union START_BIT_{
	Uint32 all;
	struct START_BIT_BITS_ bit;
}START_BIT;

typedef struct SAAD_FRAME_{
//	Uint8
	union CTRLSUM_ CTRLSUM;
	union POSITION_ POSITION;
	union VELOCITY_ VELOCITY;
	union DATA_ DATA;
	union COMMAND_BYTE_ COMMAND_BYTE;
	union START_BIT_ START_BIT;
}SAAD_FRAME;

extern SAAD_FRAME saadFrame;
extern SAAD_TESTS saadTests;

#endif // SCIDATA_H






















