<script>
    import {forward, speed, sendSpeedUpCommand, sendSpeedDownCommand} from "./stores";



    function reverseButton() {
        $forward = false;
    }

    function forwardButton() {
        $forward = true;
    }

</script>

<svg width=500 height=300 class="fixed">
    <rect width=500 height=300 style="fill:none;stroke-width:10;stroke:dodgerblue"/>
</svg>

<div class="text-3xl font-bold text-blue-400 ml-[20px] mt-[20px] fixed">
    Speed Setpoint (mph)
    </div>

<button class="btn btn-success fixed w-[125px] h-[125px] mt-[80px] ml-[330px] border-8" on:click={sendSpeedUpCommand}>
    <svg xmlns="http://www.w3.org/2000/svg" class="h-20 w-20" fill="none" viewBox="0 0 24 24" stroke="currentColor" stroke-width="2">
        <path stroke-linecap="round" stroke-linejoin="round" d="M12 4v16m8-8H4" />
    </svg>
</button>

<button class="btn btn-error fixed w-[125px] h-[125px] mt-[80px] ml-[30px] border-8" on:click={sendSpeedDownCommand}>
    <svg xmlns="http://www.w3.org/2000/svg" class="h-20 w-20" fill="none" viewBox="0 0 24 24" stroke="currentColor" stroke-width="2">
        <path stroke-linecap="round" stroke-linejoin="round" d="M20 12H4" />
    </svg>
</button>

<button class="btn {((!$forward) ? '' : 'btn-outline')} btn-primary fixed w-[125px] h-[65px] mt-[220px] ml-[70px] border-4 text-2xl font-bold" on:click={reverseButton}>
   REV
</button>

<div class="text-7xl mt-[100px] ml-[203px] fixed font-bold text-primary">
    {(($speed.toString().length<=1) ? "0"+$speed.toString() : $speed.toString())}
</div>

<button class="btn {(($forward) ? '' : 'btn-outline')} btn-primary fixed w-[125px] h-[65px] mt-[220px] ml-[290px] border-4 text-2xl font-bold" on:click={forwardButton}>
    FWD
</button>