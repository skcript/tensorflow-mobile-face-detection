package org.tensorflow.demo;

import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;
import android.widget.ImageView;
import android.widget.TextView;

public class Splash extends AppCompatActivity {

    private TextView intro;
    private ImageView logo;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_splash);
        intro=(TextView) findViewById(R.id.intro);
        logo=(ImageView) findViewById(R.id.logo);
        Animation myanim= AnimationUtils.loadAnimation(this,R.anim.mytransition);
        intro.startAnimation(myanim);
        logo.startAnimation(myanim);
        final Intent i=new Intent(this,DetectorActivity.class);
        Thread timer=new Thread(){
            public void run(){
                try{
                    sleep(2000);
                }
                catch(InterruptedException e){
                    e.printStackTrace();
                }
                finally {
                    startActivity(i);
                    finish();
                }
            }
        };
        timer.start();
    }
}
