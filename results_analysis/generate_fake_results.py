#initial timestamp in seconds, used as test ID as well
starting_timestamp=1731956299

missed_stops=[0,19,20,39]

test_ID=str(starting_timestamp)

#timestamp variable that can be incremented
current_timestamp=starting_timestamp

#headers and first row
column_headers="Test ID,POST Type,Server Timestamp,Client Timestamp,Stop Number,Arrive or Depart\n"
first_row=test_ID+",robot,"+str(starting_timestamp)+","+str(starting_timestamp)+",0,arrive\n"

#write headers and first row to csv
f=open("fake_results.csv","w")
f.write(column_headers)
f.write(first_row)
f.close()

for stop_number in range(50):

    f=open("fake_results.csv","a")

    #generate fake POST requests
    current_timestamp+=1
    departure_POST=test_ID+",robot,"+str(current_timestamp)+","+str(current_timestamp)+","+str(stop_number)+",depart\n"
    f.write(departure_POST)

    current_timestamp+=1
    if stop_number in missed_stops:
        pass
    else:
        LIC_POST=test_ID+",LIC,"+str(current_timestamp)+","+str(current_timestamp)+",0,N/A\n"
        f.write(LIC_POST)

    current_timestamp+=1
    arrival_POST=test_ID+",robot,"+str(current_timestamp)+","+str(current_timestamp)+","+str(stop_number+1)+",arrive\n"
    f.write(arrival_POST)
    f.close()