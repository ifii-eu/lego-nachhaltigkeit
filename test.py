listt1 = [0, 1, 2, 3, 4, 5, 6, 7 ]

 

listt2 = listt1.copy()

 

print ("The new list is : " + str(listt2))

 

# Adding a new element to new list

listt2.append(33)

 

# Printing the list after adding a new element

print ("The new list : " + str(listt2))

#As we are creating a shallow copy there will be no change in the old list

print ("The old list : " + str(listt1))