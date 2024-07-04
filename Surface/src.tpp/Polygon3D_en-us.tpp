topic "";
[i448;a25;kKO9;2 $$1,0#37138531426314131252341829483380:class]
[l288;2 $$2,2#27521748481378242620020725143825:desc]
[0 $$3,0#96390100711032703541132217272105:end]
[H6;0 $$4,0#05600065144404261032431302351956:begin]
[i448;a25;kKO9;2 $$5,0#37138531426314131252341829483370:item]
[l288;a4;*@5;1 $$6,6#70004532496200323422659154056402:requirement]
[l288;i1121;b17;O9;~~~.1408;2 $$7,0#10431211400427159095818037425705:param]
[i448;b42;O9;2 $$8,8#61672508125594000341940100500538:tparam]
[b42;2 $$9,9#13035079074754324216151401829390:normal]
[2 $$0,0#00000000000000000000000000000000:Default]
[{_} 
[ {{10000@(113.42.0) [s0;%% [*@7;4 Polygon3D]]}}&]
[s0; &]
[s0; Functions for 3D Point3D concave and convex flat polygons. No 
specific order is required.&]
[s0; &]
[ {{10000F(128)G(128)@1 [s0;%% [* Function List]]}}&]
[s4; &]
[s0; [@(0.0.255) enum] ContainsPointRes [* ContainsPoint]([@(0.0.255) const] 
Vector<Point3D>[@(0.0.255) `&] [*@3 polygon], [@(0.0.255) const] Point3D[@(0.0.255) `&] 
[*@3 point], [@(0.0.255) double] [*@3 distanceTol], [@(0.0.255) double] 
[*@3 angleNormalTol])&]
[s2;%% Checks if point [%-*@3 point] is inside or in the boundaries 
of polygon [%-*@3 polygon].&]
[s2;%% [%-*@3 distanceTol] indicates the tolerance in m to check if 
a point is in the plane of the [%-*@3 polygon].&]
[s2;%% [%-*@3 angleNormalTol] indicates the tolerance in rad of the 
normals of the triangles of the [%-*@3 polygon], to test if [%-*@3 polygon 
]is flat.&]
[s2; [%% Returned ]ContainsPointRes is:&]
[s2;i150;O0; >0 (POLY`_IN) if [*@3 pt] is inside&]
[s2;i150;O0; `=`= 0 (POLY`_SECT) if [*@3 pt] is in the boundary. &]
[s2; Negatives values indicate that [*@3 point ]is outside:&]
[s2;i150;O0; POLY`_NOPLAN `= `-4. The polygon is not flat.&]
[s2;i150;O0; POLY`_FAR `= `-3. The point is not in the same plane.&]
[s2;i150;O0; POLY`_3 `= `-2. The polygon has less than three points.&]
[s2;i150;O0; POLY`_OUT `= `-1. The point is in the same plane, but 
outside.&]
[s3; &]
[s4; &]
[s5;:Upp`:`:Area`(const UVector`&`): [@(0.0.255) double] [* Area]([@(0.0.255) const] 
UVector<Point3D>[@(0.0.255) `&] [*@3 p])&]
[s2;%% Returns the area of polygon [%-*@3 p].&]
[s3; &]
[s4; &]
[s5;:Upp`:`:IsRectangle`(const UVector`&`): [@(0.0.255) bool] [* IsRectangle]([@(0.0.255) c
onst] UVector<Point3D>[@(0.0.255) `&] [*@3 p])&]
[s2;%% Returns true if [%-*@3 p] is a rectangle (flat).&]
[s3; &]
[s4; &]
[s5;:Upp`:`:IsFlat`(const UVector`&`): [@(0.0.255) bool] [* IsFlat]([@(0.0.255) const] 
UVector<Point3D>[@(0.0.255) `&] [*@3 p])&]
[s2;%% Returns true if the polygon [%-*@3 p] is on a flat surface.&]
[s3; &]
[s4; &]
[s5;:Upp`:`:Centroid`(const UVector`&`): Point3D [* Centroid]([@(0.0.255) const] 
UVector<Point3D>[@(0.0.255) `&] [*@3 p])&]
[s2;%% Returns the centroid of polygon [%-*@3 p].&]
[s3; &]
[s4; &]
[s5;:Upp`:`:Point3Dto2D`_XY`(const Vector`&`): Vector<Pointf> [* Point3Dto2D`_XY]([@(0.0.255) c
onst] Vector<Point3D>[@(0.0.255) `&] [*@3 bound])&]
[s2;%% Converts the 3D polygon [%-*@3 bound] to a 2D polygon based 
on X and Y.&]
[s3; &]
[s4; &]
[s5;:Upp`:`:Point3Dto2D`_XZ`(const Vector`&`): Vector<Pointf> [* Point3Dto2D`_XZ]([@(0.0.255) c
onst] Vector<Point3D>[@(0.0.255) `&] [*@3 bound])&]
[s2;%% Converts the 3D polygon [%-*@3 bound] to a 2D polygon based 
on X and Z.&]
[s3; &]
[s4; &]
[s5;:Upp`:`:Point3Dto2D`_YZ`(const Vector`&`): Vector<Pointf> [* Point3Dto2D`_YZ]([@(0.0.255) c
onst] Vector<Point3D>[@(0.0.255) `&] [*@3 bound])&]
[s2;%% Converts the 3D polygon [%-*@3 bound] to a 2D polygon based 
on Y and Z.&]
[s3; &]
[s4; &]
[s5;:Upp`:`:Point2Dto3D`_XY`(const Vector`&`): Vector<Point3D> [* Point2Dto3D`_XY]([@(0.0.255) c
onst] Vector<Pointf>[@(0.0.255) `&] [*@3 bound])&]
[s2;%% Converts the 2D polygon [%-*@3 bound] to a 3D polygon based 
on X and Y.&]
[s3; &]
[s4; &]
[s5;:Upp`:`:Point2Dto3D`_XZ`(const Vector`&`): Vector<Point3D> [* Point2Dto3D`_XZ]([@(0.0.255) c
onst] Vector<Pointf>[@(0.0.255) `&] [*@3 bound])&]
[s2;%% Converts the 2D polygon [%-*@3 bound] to a 3D polygon based 
on X and Z.&]
[s3; &]
[s4; &]
[s5;:Upp`:`:Point2Dto3D`_YZ`(const Vector`&`): Vector<Point3D> [* Point2Dto3D`_YZ]([@(0.0.255) c
onst] Vector<Pointf>[@(0.0.255) `&] [*@3 bound])&]
[s2;%% Converts the 2D polygon [%-*@3 bound] to a 3D polygon based 
on Y and Z.&]
[s3; &]
[s0;%% ]]