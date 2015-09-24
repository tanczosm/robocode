package ja;

	
	public class StatsTracker
	{
		public static final int segment_length = 100;	
	
		public GameStats[] segmentedStats = new GameStats[1000/segment_length];
		public GameStats overallStat = new GameStats();
		
		
		public StatsTracker ()
		{
			for (int i = 0; i < segmentedStats.length; i++)
			{
				segmentedStats[i] = new GameStats();
				segmentedStats[i]._distance = i * segment_length;
			}
		}
		
		public GameStats getOverallStat ()
		{
			return overallStat;	
		}
		
		public GameStats getSegmentedStat (int distance)
		{
			return segmentedStats[distance / segment_length];
		}
		
		public void add (boolean hit, int distance)
		{
			
			System.out.println(hit ? "HIT" : "MISS");
			overallStat.add(hit, distance);
			for (GameStats gs : segmentedStats)
			{
				if (distance >= gs._distance && distance <= (gs._distance+segment_length))
				{
					gs.add(hit, distance);
				}
			}
		}
		
		public double getBestDistance ()
		{
			double bestRatio = 0;
			double bestDistance = 0;
			
			for (GameStats gs : segmentedStats)
			{
				if (gs.getHitRatio() > bestRatio)
				{
					bestRatio = gs.getHitRatio();
					bestDistance = gs.getAverageDistance();
				}
			}
				
			return bestDistance;
		}
	}