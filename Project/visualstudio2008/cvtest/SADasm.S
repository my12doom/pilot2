/*
 * ARM memcpy asm replacement.
 *
 * Copyright (C) 2009 Bluush Dev Team.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */


#define _LABEL(f)   f :


.global SAD16x16_ARM



	.code 32


/*
 * Fast copy n bytes from source p2 to destination p1.
 *
 *    int SAD16x16_ARM(void *pdata1, void *pdata2, int stride)
 */

_LABEL(SAD16x16_ARM)
        stmdb    sp!,{lr}    	/* push return address */
	stmdb    sp!,{r4-r12} 	/* save registers on stack */
	mov r4,r0
	mov r6,#16
	vmov.I16 q3,#0
	vmov.I16 q4,#0

/*
 * The two strings do not end on word boundaries.
 * Copy the remaining data one byte at a time.
 */
_LABEL(ByteSerial)
	vld1.8 {d0,d1},[r4] //p1
	vld1.8 {d2,d3},[r1] //p2
	vabal.u8 q3,d0,d2
	vabal.u8 q4,d1,d3

	add r4,r2 //p+=stride
	add r1,r2 //p1+=stride
	subs     r6,r6,#1 
	bne      ByteSerial  
	vadd.u16 q3,q4
        vpaddl.u16 q3,q3
	vpaddl.u32 q3,q3
	vadd.u32 d6,d7
	vmov.u32 r0,d6[0]
_LABEL(QUITS)

	ldmia    sp!,{r4-r12} 	/* restore registers from stack */

        ldmia    sp!,{pc}    	/* return to caller */

		.ltorg
		

 
