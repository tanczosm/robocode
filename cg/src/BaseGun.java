package cg;

import java.util.ArrayList;
import java.awt.geom.Rectangle2D;

public abstract class BaseGun {
	
	ArrayList<VirtualBullet> _virtualShots = new ArrayList<VirtualBullet>();
	private Averager _averager = new Averager(200);

	// Add a situation that has been fully registered (ie. the shot was corrected)
	abstract void addSituation(Situation s);
	abstract String getName ();
	abstract void update();
	abstract double projectBearing(Situation Situation, double x, double y, double enemyHeading);


	public void checkVirtualBullets(long time, Rectangle2D.Double enemyBox){		
		
		for(int i = _virtualShots.size() - 1; i >= 0; i--){
			VirtualBullet vb = _virtualShots.get(i);
			double d = vb.TheSituation.getDistance(time);
			double heading = vb.TheSituation.EnemyHeading + vb.TheBearing;
			double nextX = vb.TheSituation.RX + Math.sin(heading) * d;
			double nextY = vb.TheSituation.RY + Math.cos(heading) * d;
			
			if(enemyBox.contains(nextX, nextY)){
				_averager.addValue(1d);
				_virtualShots.remove(i);
			}
		}
	}


	public double getRatingPercent(){
		return _averager.getAverage();
	}

	public void takeVirtualShot(Situation s, double bearing){
		_virtualShots.add(new VirtualBullet(s, bearing));
	}
	
	public void removePassed(Situation s){
		for(int i = _virtualShots.size() - 1; i >= 0; i--){
			if((_virtualShots.get(i)).TheSituation == s){
				_virtualShots.remove(i);
				_averager.addValue(0d);
				break;
			}
		}
	}
	
	public void clear(){
		_virtualShots.clear();
	}
	
	class VirtualBullet{
		public Situation TheSituation;
		public double TheBearing;
		
		VirtualBullet(Situation s, double bearing){
			TheSituation = s;
			TheBearing = bearing;
		}
	}
}
