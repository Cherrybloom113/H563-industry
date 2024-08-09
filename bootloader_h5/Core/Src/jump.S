                AREA    |.text|, CODE, READONLY

; Reset Handler

start_app  	PROC
                EXPORT  start_app  
				;set vector r0(0x08040000)-->VTOR¼Ä´æÆ÷
				LDR R1, =0XE000ED08
				STR R0, [R1]
				
				;set sp
				LDR R1, [R0]
				MOV SP, R1
				
				;jump
				LDR R1, [R0, #4]
				BX R1
				
                ENDP
				END