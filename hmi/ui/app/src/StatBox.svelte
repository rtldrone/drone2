<script>
    import {currentSpeed, distance, draw, voltage, batteryState, drawState, distanceState} from "./stores";

    let nf = new Intl.NumberFormat('en-US', { minimumIntegerDigits: 2, minimumFractionDigits: 2, maximumFractionDigits: 2});
</script>

<div class="stats stats-vertical lg:stats-horizontal shadow font-black">

    <div class="stat">
        <div class="stat-title">Actual Speed</div>
        <div class="stat-value">{nf.format($currentSpeed)} mph</div>
    </div>

    <div class="stat">
        <div class="stat-title">Battery Voltage</div>
        <div class="stat-value {(($batteryState == 0) ? 'text-green-500' : ($batteryState == 1) ? 'text-yellow-500' : 'text-red-500')}">{nf.format($voltage)} V</div>
    </div>

    <div class="stat">
        <div class="stat-title">Current Draw</div>
        <div class="stat-value {(($drawState == 0) ? 'text-green-500' : ($drawState == 1) ? 'text-yellow-500' : 'text-red-500')}">{nf.format($draw)} A</div>
    </div>

    <div class="stat">
        <div class="stat-title">Sensor Distance</div>
        <div class="stat-value {(($distanceState == 0) ? 'text-green-500' : ($distanceState == 1) ? 'text-yellow-500' : 'text-red-500')}">{nf.format($distance)} ft</div>
    </div>

</div>