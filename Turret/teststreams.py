"""
read numbers till e.o.f. and show squares
"""
def interact():
	print('Hello stream world!')
	while True:
		try:
			reply = input('Enter a number: ')
		except EOFError:
			break
		else:
			num = int(reply)
			print(num.encode())
	print('Bye!')
	
if __name__=='__main__':
	interact()
	
	
