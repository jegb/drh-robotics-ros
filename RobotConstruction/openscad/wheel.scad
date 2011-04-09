wheel();

module wheel()
{
	$fn = 50;
	inchTomm = 25.4;
	outerDiameter = 3 * inchTomm;
	outerRimHeight = 0.23 * inchTomm;
	innerDiameter = outerDiameter - 2 * 0.2 * inchTomm; 

	mountingHolesCircleDiameter = 0.625 * inchTomm; 

	difference()
	{
		cylinder(h = outerRimHeight, r=outerDiameter/2, center=true);
		translate([0, 0, 1.4]) cylinder(h = 6, r=innerDiameter/2, center=false);
		translate([0, 0, -6-1.4]) cylinder(h = 6, r=innerDiameter/2, center=false);
		cylinder(h = 7, r=9.52/2, center=true);

		for (theta = [0, 90, 180, 270])
		{
			rotate( theta, [0, 0, 1])
			{
				translate([mountingHolesCircleDiameter / 2, 0, 0] )
					cylinder(h = 7, r=3.2/2, center=true);
				rotate( 45, [0, 0, 1])
					translate([mountingHolesCircleDiameter / 2, 0, 0] )
						cylinder(h = 7, r=2.4/2, center=true);
			}
		}
	}
}