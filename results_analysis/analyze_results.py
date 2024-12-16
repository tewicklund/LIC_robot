from analysis_functions import *

#ask user for test ID, should be 1st column of results csv
#test_ID=input("Enter test ID: ")
test_ID='LIC 2 Dec 16 2024'

grid_title='LIC on table results grid'

#list to store LIC posts
LIC_post_list=csv_to_LIC_post_list(test_ID)

#for post in LIC_post_list:
#    print(post)

num_squares=70
grid_chars=['O']*num_squares

for grid_square in range(num_squares):
    start_timestamp=get_timestamp_from_post(str(grid_square),'depart',LIC_post_list)
    end_timestamp=get_timestamp_from_post(str(grid_square+1),'arrive',LIC_post_list)

    for post in LIC_post_list:
        if start_timestamp != 0 and end_timestamp != 0:
            
            if post.post_type=='LIC Trigger POST' and start_timestamp<post.server_timestamp and post.server_timestamp<end_timestamp:
                grid_chars[grid_square]='X'

export_grid_as_png(grid_title,grid_chars)

