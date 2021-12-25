# solenoid_driver_node

launchファイルにて
```
<node pkg="solenoid_driver_node" type="solenoid_driver_node" name="auaua" output="screen">
      <param name="bid" type="string "value="4a0" />
      <param name="name" value="auaua" />
</node>
```
と，bidと使用する電磁弁を表す名前をつけてあげること．
雑な作りなのでlaunchでnameをつけてあげないと落ちます．(まあ上記以外の使い方はしないと思いますが…)
 
