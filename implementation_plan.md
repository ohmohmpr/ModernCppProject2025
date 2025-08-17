# Grid Mapping

## 0. CMakeLists.txt

use fetch content might be a better idea.

## 1. Iteration/Pipeline

use std::file to read all data in Data directory  
ANSWER = Dataset()  
and count the number of files.  
ANSWER = dataset.size()  

--> ranges_raw and poses_raw

## 2. Occupation mapping Data Structure

--> Voxelhash map? split between map algorithm and voxel map.  
--> inverse_sensor_model  
--> log_odd  

## 3. Map_resolution

--> default ? lets use 0.01m

## 4. Prob_occ, prob_free, prior

--> map_res = 0.25  
--> prior = 0.50  
--> prob_occ = 0.90  
--> prob_free = 0.35  

## remarked

-->lambda function
