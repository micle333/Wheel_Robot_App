package com.example.kortium;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

public class JoystickView extends View {

    private Paint paint;
    private float centerX, centerY;
    private float joystickRadius;
    private float touchX, touchY;
    private boolean isMoving;
    private long lastMoveTime;
    private static final long MIN_MOVE_DURATION_MS = 0; // Минимальная продолжительность для регистрации движения
    private static final float JOYSTICK_RADIUS = 100; // Радиус видимого джойстика
    private static final float BACKGROUND_RADIUS = 200; // Расширенная область джойстика
    private static final float SENSITIVE_AREA_RADIUS = 300; // Чувствительная область

    private JoystickListener joystickListener;

    public JoystickView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        paint = new Paint();
        paint.setAntiAlias(true);
        joystickRadius = JOYSTICK_RADIUS;
        isMoving = false;
        lastMoveTime = 0;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        centerX = getWidth() / 2;
        centerY = getHeight() / 2;

        // Рисуем фоновый круг джойстика (больше, без контура)
        paint.setColor(Color.TRANSPARENT);
        paint.setStyle(Paint.Style.FILL);
        canvas.drawCircle(centerX, centerY, BACKGROUND_RADIUS, paint);

        // Рисуем видимый круг джойстика (прозрачный с серым контуром)
        paint.setColor(Color.TRANSPARENT);
        paint.setStyle(Paint.Style.FILL);
        canvas.drawCircle(centerX, centerY, joystickRadius, paint);

        paint.setColor(Color.GRAY);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(4);
        canvas.drawCircle(centerX, centerY, joystickRadius, paint);

        // Рисуем ползунок (белый)
        if (isMoving) {
            float distanceX = touchX - centerX;
            float distanceY = touchY - centerY;
            float distance = (float) Math.sqrt(distanceX * distanceX + distanceY * distanceY);

            // Ограничиваем движение ползунка по окружности
            if (distance > BACKGROUND_RADIUS - joystickRadius / 2) {
                float scale = (BACKGROUND_RADIUS - joystickRadius / 2) / distance;
                touchX = centerX + distanceX * scale;
                touchY = centerY + distanceY * scale;
            }

            paint.setColor(Color.WHITE);
            paint.setStyle(Paint.Style.FILL);
            canvas.drawCircle(touchX, touchY, joystickRadius / 2, paint);
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        float x = event.getX();
        float y = event.getY();

        long currentTime = System.currentTimeMillis();

        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
                // Проверяем, что касание произошло в чувствительной области
                float distanceFromCenter = (float) Math.sqrt((x - centerX) * (x - centerX) + (y - centerY) * (y - centerY));
                if (distanceFromCenter <= SENSITIVE_AREA_RADIUS) {
                    isMoving = true;
                    touchX = x;
                    touchY = y;
                    lastMoveTime = currentTime;
                    invalidate(); // Перерисовать View
                }
                break;

            case MotionEvent.ACTION_MOVE:
                // Проверяем, что прошло достаточно времени с последнего движения
                if (isMoving && (currentTime - lastMoveTime) > MIN_MOVE_DURATION_MS) {
                    float distanceX = x - centerX;
                    float distanceY = y - centerY;
                    float distance = (float) Math.sqrt(distanceX * distanceX + distanceY * distanceY);

                    // Ограничиваем движение ползунка по окружности
                    if (distance <= BACKGROUND_RADIUS) {
                        touchX = x;
                        touchY = y;
                    } else if (distance > BACKGROUND_RADIUS - joystickRadius / 2) {
                        float scale = (BACKGROUND_RADIUS - joystickRadius / 2) / distance;
                        touchX = centerX + distanceX * scale;
                        touchY = centerY + distanceY * scale;
                    }

                    invalidate(); // Перерисовать View
                    if (joystickListener != null) {
                        joystickListener.onJoystickMove(touchX, touchY);
                    }

                    lastMoveTime = currentTime;
                }
                break;

            case MotionEvent.ACTION_UP:
                isMoving = false;
                invalidate(); // Перерисовать View
                if (joystickListener != null) {
                    joystickListener.onJoystickRelease();
                }
                break;
        }
        return true;
    }

    public void setJoystickListener(JoystickListener listener) {
        this.joystickListener = listener;
    }

    public interface JoystickListener {
        void onJoystickMove(float x, float y);
        void onJoystickRelease();
    }
}
