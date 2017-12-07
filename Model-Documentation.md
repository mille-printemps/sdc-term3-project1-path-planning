# Path planning project

## Model

The model is composed of attributes of lanes derived from the sensor inputs and these attributes are used to make decision to move from a lane to another.

|Attributes |Evaluation Criteria|
|:---------|:----------|
|The velocity of the vehicle ahead on the same lane|The faster, the better |
| The distance to the vehicle ahead on the same lane| The longer, the better|
| The distance to the vehicle behind on adjacent lane(s)| The longer, the better|
| The difference of the velocity from the vehicle behind on adjacent lane(s)|  The greater, the better|

Each attribute is derived at every input of the sensor, and a cost is calculated for each attribute so that the evaluation criteria above can reflect to the cost. For example, if the difference of the velocity from the vehicle behind on an adjacent lane is high, a lower cost is calculated because the possibility of collision with the vehicle on the adjacent lane is supposed to be lower when you move to that lane.

The criteria to change lanes are  **1)** the total cost of the attributes of the lane is lower than a value configured, **2)** the total cost of the attributes of the lane is lower than the one of the current lane, **3)** the difference between the costs of the attributes of the lanes is higher than a value configured, **4)** no vehicles are around within a certain distance in the lane and **5)** the current velocity of the vehicle is faster than a criterion. 

The 2nd and the 3rd criteria are for avoiding frequent changes of the lane, and the 4th and 5th criteria are for moving into the lane safely. 

## Discussion

Though the model and algorithm is not A<sup>*</sup> search and that ilk, it is found that making decision to change lanes with some heuristic cost functions works most of cases. However, one issue is that, since a minimum velocity is included in the criteria for the lane change, the vehicle sometimes gets stuck in a slow lane even when the costs of the other lanes are lower than the current lane. As long as I tried, such a minimum velocity for changing lanes was necessary to reduce possibility of collision.

On the flip side of the coin, keeping a velocity and a lane are properties to make the vehicle do a safe drive. To mitigate the issue, the vehicle would need to behave more adaptively depending on the status of the other lanes, not just on the fixed criteria.

No concept of time is included in the model and algorithm at this point. Analyzing and predict behaviors of other vehicles with time series data would be a next step for improvement.



