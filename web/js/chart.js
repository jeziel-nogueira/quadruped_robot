/**
 * Custom High-Performance Canvas-based Time Series Plotter
 * Zero external dependencies. Extremely lightweight.
 */

class TimeSeriesPlot {
    constructor(canvas, maxDataPoints = 120, colors = ["#00f5d4", "#00bbf9", "#f15bb5"]) {
        this.canvas = canvas;
        this.ctx = canvas.getContext("2d");
        this.maxDataPoints = maxDataPoints;
        this.colors = colors;
        
        // Data structure: Array of arrays (one per series)
        this.series = [];
        this.labels = [];
    }

    addPoint(values, label = "") {
        // Initialize series if not yet done
        if (this.series.length === 0) {
            this.series = values.map(() => []);
        }

        // Add values to each series
        for (let i = 0; i < values.length; i++) {
            if (i < this.series.length) {
                this.series[i].push(values[i]);
                if (this.series[i].length > this.maxDataPoints) {
                    this.series[i].shift();
                }
            }
        }
        
        this.labels.push(label);
        if (this.labels.length > this.maxDataPoints) {
            this.labels.shift();
        }

        this.render();
    }

    render() {
        const ctx = this.ctx;
        const clientW = this.canvas.clientWidth || 350;
        const clientH = this.canvas.clientHeight || 120;
        const width = this.canvas.width = clientW * window.devicePixelRatio;
        const height = this.canvas.height = clientH * window.devicePixelRatio;
        
        ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
        const drawWidth = clientW;
        const drawHeight = clientH;

        // Clear canvas
        ctx.clearRect(0, 0, drawWidth, drawHeight);

        if (this.series.length === 0 || this.series[0].length === 0) {
            return;
        }

        // Determine min and max across all series for scaling
        let allValues = this.series.flat();
        let min = Math.min(...allValues);
        let max = Math.max(...allValues);
        
        // Add padding to y-scale
        let range = max - min;
        if (range < 0.1) range = 1.0;
        min -= range * 0.15;
        max += range * 0.15;
        range = max - min;

        // Draw grid lines
        ctx.strokeStyle = "rgba(255, 255, 255, 0.05)";
        ctx.lineWidth = 1;
        const numGridLines = 4;
        for (let i = 0; i <= numGridLines; i++) {
            const y = (i / numGridLines) * (drawHeight - 20) + 10;
            ctx.beginPath();
            ctx.moveTo(35, y);
            ctx.lineTo(drawWidth - 10, y);
            ctx.stroke();

            // Draw Y-axis labels
            const gridVal = max - (i / numGridLines) * range;
            ctx.fillStyle = "#8e9bb0";
            ctx.font = "8px 'Outfit', sans-serif";
            ctx.fillText(gridVal.toFixed(1), 5, y + 3);
        }

        // Draw each series line
        const numPoints = this.series[0].length;
        const paddingLeft = 35;
        const paddingRight = 10;
        const paddingTop = 10;
        const paddingBottom = 10;
        const plotWidth = drawWidth - paddingLeft - paddingRight;
        const plotHeight = drawHeight - paddingTop - paddingBottom;

        for (let s = 0; s < this.series.length; s++) {
            const data = this.series[s];
            const color = this.colors[s % this.colors.length];
            
            ctx.strokeStyle = color;
            ctx.lineWidth = 1.5;
            ctx.beginPath();

            for (let i = 0; i < data.length; i++) {
                const x = paddingLeft + (i / (this.maxDataPoints - 1)) * plotWidth;
                const val = data[i];
                // Map val to y pixel
                const y = paddingTop + (1.0 - (val - min) / range) * plotHeight;

                if (i === 0) {
                    ctx.moveTo(x, y);
                } else {
                    ctx.lineTo(x, y);
                }
            }
            ctx.stroke();

            // Subtle glow effect
            ctx.save();
            ctx.strokeStyle = color;
            ctx.lineWidth = 3;
            ctx.shadowColor = color;
            ctx.shadowBlur = 8;
            ctx.globalAlpha = 0.25;
            ctx.stroke();
            ctx.restore();
        }
    }
}
